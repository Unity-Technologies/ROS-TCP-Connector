using RosMessageTypes.RosTcpEndpoint;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using UnityEngine.Serialization;

namespace Unity.Robotics.ROSTCPConnector
{
    public class ROSConnection : MonoBehaviour
    {
        // Variables required for ROS communication
        [FormerlySerializedAs("hostName")] public string rosIPAddress = "127.0.0.1";
        [FormerlySerializedAs("hostPort")] public int rosPort = 10000;

        [Tooltip("If blank, determine IP automatically.")]
        public string overrideUnityIP = "";

        public int unityPort = 5005;
        bool alreadyStartedServer = false;

        private int networkTimeout = 2000;

        [Tooltip("While waiting for a service to respond, check this many times before giving up.")]
        public int awaitDataMaxRetries = 10;

        [Tooltip("While waiting for a service to respond, wait this many seconds between checks.")]
        public float awaitDataSleepSeconds = 1.0f;

        [Tooltip("While reading received messages, read this many bytes at a time.")]
        public int readChunkSize = 2048;

        [Tooltip("While waiting to read a full message, check this many times before giving up.")]
        public int awaitDataReadRetry = 10;

        static object _lock = new object(); // sync lock 
        static List<Task> activeConnectionTasks = new List<Task>(); // pending connections

        const string ERROR_TOPIC_NAME = "__error";
        const string SYSCOMMAND_TOPIC_NAME = "__syscommand";
        const string HANDSHAKE_TOPIC_NAME = "__handshake";

        const string SYSCOMMAND_SUBSCRIBE = "subscribe";
        const string SYSCOMMAND_PUBLISH = "publish";

        // GUI window variables
        internal HUDPanel hudPanel = null;

        public bool showHUD = true;

        struct SubscriberCallback
        {
            public ConstructorInfo messageConstructor;
            public List<Func<Message, Message>> callbacks;
        }

        Dictionary<string, SubscriberCallback> subscribers = new Dictionary<string, SubscriberCallback>();

        public void Subscribe<T>(string topic, Action<T> callback) where T : Message, new()
        {
            SubscriberCallback subCallbacks;
            if (!subscribers.TryGetValue(topic, out subCallbacks))
            {
                subCallbacks = new SubscriberCallback
                {
                    messageConstructor = typeof(T).GetConstructor(new Type[0]),
                    callbacks = new List<Func<Message, Message>> { }
                };
                subscribers.Add(topic, subCallbacks);
            }

            subCallbacks.callbacks.Add((Message msg) =>
            {
                callback((T) msg);
                return null;
            });
        }

        public void ImplementService<T>(string topic, Func<T, Message> callback)
            where T : Message, new()
        {
            SubscriberCallback subCallbacks;
            if (!subscribers.TryGetValue(topic, out subCallbacks))
            {
                subCallbacks = new SubscriberCallback
                {
                    messageConstructor = typeof(T).GetConstructor(new Type[0]),
                    callbacks = new List<Func<Message, Message>> { }
                };
                subscribers.Add(topic, subCallbacks);
            }

            subCallbacks.callbacks.Add((Message msg) => { return callback((T) msg); });
        }

        public async void SendServiceMessage<RESPONSE>(string rosServiceName, Message serviceRequest,
            Action<RESPONSE> callback) where RESPONSE : Message, new()
        {
            // Serialize the message in service name, message size, and message bytes format
            byte[] messageBytes = GetMessageBytes(rosServiceName, serviceRequest);

            TcpClient client = new TcpClient();
            await client.ConnectAsync(rosIPAddress, rosPort);

            NetworkStream networkStream = client.GetStream();
            networkStream.ReadTimeout = networkTimeout;

            RESPONSE serviceResponse = new RESPONSE();

            int serviceID = 0;

            // Send the message
            try
            {
                if (hudPanel != null) serviceID = hudPanel.AddServiceRequest(rosServiceName, serviceRequest);
                networkStream.Write(messageBytes, 0, messageBytes.Length);
            }
            catch (Exception e)
            {
                Debug.LogError("SocketException: " + e);
                goto finish;
            }

            if (!networkStream.CanRead)
            {
                Debug.LogError("Sorry, you cannot read from this NetworkStream.");
                goto finish;
            }

            // Poll every 1 second(s) for available data on the stream
            int attempts = 0;
            while (!networkStream.DataAvailable && attempts <= this.awaitDataMaxRetries)
            {
                if (attempts == this.awaitDataMaxRetries)
                {
                    Debug.LogError("No data available on network stream after " + awaitDataMaxRetries + " attempts.");
                    goto finish;
                }

                attempts++;
                await Task.Delay((int) (awaitDataSleepSeconds * 1000));
            }

            try
            {
                string serviceName;
                (string topicName, byte[] content) = await ReadMessageContents(networkStream);
                serviceResponse.Deserialize(content, 0);
            }
            catch (Exception e)
            {
                Debug.LogError("Exception raised!! " + e);
            }

            finish:
            callback(serviceResponse);
            if (hudPanel != null) hudPanel.AddServiceResponse(serviceID, serviceResponse);
            if (client.Connected)
                client.Close();
        }

        public void RegisterSubscriber(string topic, string rosMessageName)
        {
            SendSysCommand(SYSCOMMAND_SUBSCRIBE,
                new SysCommand_Subscribe {topic = topic, message_name = rosMessageName});
        }

        public void RegisterPublisher(string topic, string rosMessageName)
        {
            SendSysCommand(SYSCOMMAND_PUBLISH, new SysCommand_Publish {topic = topic, message_name = rosMessageName});
        }

        private static ROSConnection _instance;

        public static ROSConnection instance
        {
            get
            {
                if (_instance == null)
                {
                    GameObject prefab = Resources.Load<GameObject>("ROSConnectionPrefab");
                    if (prefab == null)
                    {
                        Debug.LogWarning(
                            "No settings for ROSConnection.instance! Open \"ROS Settings\" from the Robotics menu to configure it.");
                        GameObject instance = new GameObject("ROSConnection");
                        _instance = instance.AddComponent<ROSConnection>();
                    }
                    else
                    {
                        Instantiate(prefab);
                    }
                }

                return _instance;
            }
        }

        void OnEnable()
        {
            if (_instance == null)
                _instance = this;
        }

        private void Start()
        {
            InitializeHUD();
            Subscribe<RosUnityError>(ERROR_TOPIC_NAME, RosUnityErrorCallback);

            if (overrideUnityIP != "")
            {
                StartMessageServer(overrideUnityIP, unityPort); // no reason to wait, if we already know the IP
            }

            SendServiceMessage<UnityHandshakeResponse>(HANDSHAKE_TOPIC_NAME,
                new UnityHandshakeRequest(overrideUnityIP, (ushort) unityPort), RosUnityHandshakeCallback);
        }

        void OnValidate()
        {
            InitializeHUD();
        }

        private void InitializeHUD()
        {
            if (!Application.isPlaying || (!showHUD && hudPanel == null))
                return;

            if (hudPanel == null)
            {
                hudPanel = gameObject.AddComponent<HUDPanel>();
                hudPanel.host = $"{rosIPAddress}:{rosPort}";
            }

            hudPanel.isEnabled = showHUD;
        }

        void RosUnityHandshakeCallback(UnityHandshakeResponse response)
        {
            StartMessageServer(response.ip, unityPort);
        }

        void RosUnityErrorCallback(RosUnityError error)
        {
            Debug.LogError("ROS-Unity error: " + error.message);
        }

        /// <param name="tcpClient"></param> TcpClient to read byte stream from.
        protected async Task HandleConnectionAsync(TcpClient tcpClient)
        {
            await Task.Yield();

            // continue asynchronously on another thread
            await ReadMessage(tcpClient.GetStream());
        }

        async Task ReadMessage(NetworkStream networkStream)
        {
            if (!networkStream.CanRead)
                return;

            SubscriberCallback subs;

            (string topicName, byte[] content) = await ReadMessageContents(networkStream);

            if (!subscribers.TryGetValue(topicName, out subs))
                return; // not interested in this topic

            Message msg = (Message) subs.messageConstructor.Invoke(new object[0]);
            msg.Deserialize(content, 0);

            if (hudPanel != null)
                hudPanel.SetLastMessageReceived(topicName, msg);

            foreach (Func<Message, Message> callback in subs.callbacks)
            {
                try
                {
                    Message response = callback(msg);
                    if (response != null)
                    {
                        // if the callback has a response, it's implementing a service
                        WriteDataStaggered(networkStream, topicName, response);
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError("Subscriber callback problem: " + e);
                }
            }
        }

        async Task<Tuple<string, byte[]>> ReadMessageContents(NetworkStream networkStream)
        {
            // Get first bytes to determine length of topic name
            byte[] rawTopicBytes = new byte[4];
            networkStream.Read(rawTopicBytes, 0, rawTopicBytes.Length);
            int topicLength = BitConverter.ToInt32(rawTopicBytes, 0);

            // Read and convert topic name
            byte[] topicNameBytes = new byte[topicLength];
            networkStream.Read(topicNameBytes, 0, topicNameBytes.Length);
            string topicName = Encoding.ASCII.GetString(topicNameBytes, 0, topicLength);

            byte[] full_message_size_bytes = new byte[4];
            networkStream.Read(full_message_size_bytes, 0, full_message_size_bytes.Length);
            int full_message_size = BitConverter.ToInt32(full_message_size_bytes, 0);

            byte[] readBuffer = new byte[full_message_size];
            int bytesRemaining = full_message_size;
            int totalBytesRead = 0;

            int attempts = 0;
            // Read in message contents until completion, or until attempts are maxed out
            while (bytesRemaining > 0 && attempts <= this.awaitDataReadRetry)
            {
                if (attempts == this.awaitDataReadRetry)
                {
                    Debug.LogError("No more data to read network stream after " + awaitDataReadRetry + " attempts.");
                    return Tuple.Create(topicName, readBuffer);
                }

                // Read the minimum of the bytes remaining, or the designated readChunkSize in segments until none remain
                int bytesRead = networkStream.Read(readBuffer, totalBytesRead, Math.Min(readChunkSize, bytesRemaining));
                totalBytesRead += bytesRead;
                bytesRemaining -= bytesRead;

                if (!networkStream.DataAvailable) 
                {
                    attempts++;
                    await Task.Yield();
                }
            }
            return Tuple.Create(topicName, readBuffer);
        }

        /// <summary>
        /// 	Handles multiple connections and locks.
        /// </summary>
        /// <param name="tcpClient"></param> TcpClient to read byte stream from.
        private async Task StartHandleConnectionAsync(TcpClient tcpClient)
        {
            var connectionTask = HandleConnectionAsync(tcpClient);

            lock (_lock)
                activeConnectionTasks.Add(connectionTask);

            try
            {
                await connectionTask;
                // we may be on another thread after "await"
            }
            catch (Exception ex)
            {
                Debug.LogError(ex.ToString());
            }
            finally
            {
                lock (_lock)
                    activeConnectionTasks.Remove(connectionTask);
            }
        }

        TcpListener tcpListener;

        protected async void StartMessageServer(string ip, int port)
        {
            if (alreadyStartedServer)
                return;

            alreadyStartedServer = true;
            while (true)
            {
                try
                {
                    if (!Application.isPlaying)
                        break;
                    tcpListener = new TcpListener(IPAddress.Parse(ip), port);
                    tcpListener.Start();

                    Debug.Log("ROS-Unity server listening on " + ip + ":" + port);

                    while (true) //we wait for a connection
                    {
                        var tcpClient = await tcpListener.AcceptTcpClientAsync();

                        var task = StartHandleConnectionAsync(tcpClient);
                        // if already faulted, re-throw any error on the calling context
                        if (task.IsFaulted)
                            await task;

                        // try to get through the message queue before doing another await
                        // but if messages are arriving faster than we can process them, don't freeze up
                        float abortAtRealtime = Time.realtimeSinceStartup + 0.1f;
                        while (tcpListener.Pending() && Time.realtimeSinceStartup < abortAtRealtime)
                        {
                            tcpClient = tcpListener.AcceptTcpClient();
                            task = StartHandleConnectionAsync(tcpClient);
                            if (task.IsFaulted)
                                await task;
                        }
                    }
                }
                catch (ObjectDisposedException e)
                {
                    if (!Application.isPlaying)
                    {
                        // This only happened because we're shutting down. Not a problem.
                    }
                    else
                    {
                        Debug.LogError("Exception raised!! " + e);
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError("Exception raised!! " + e);
                }

                // to avoid infinite loops, wait a frame before trying to restart the server
                await Task.Yield();
            }
        }

        private void OnApplicationQuit()
        {
            if (tcpListener != null)
                tcpListener.Stop();
            tcpListener = null;
        }


        /// <summary>
        ///    Given some input values, fill a byte array in the desired format to use with
        ///     https://github.com/Unity-Technologies/Robotics-Tutorials/tree/master/catkin_ws/src/tcp_endpoint
        ///
        /// 	All messages are expected to come in the format of:
        /// 		first four bytes: int32 of the length of following string value
        /// 		next N bytes determined from previous four bytes: ROS topic name as a string
        /// 		next four bytes: int32 of the length of the remaining bytes for the ROS Message
        /// 		last N bytes determined from previous four bytes: ROS Message variables
        /// </summary>
        /// <param name="offset"></param> Index of where to start writing output data
        /// <param name="serviceName"></param> The name of the ROS service or topic that the message data is meant for
        /// <param name="fullMessageSizeBytes"></param> The full size of the already serialized message in bytes
        /// <param name="messageToSend"></param> The serialized ROS message to send to ROS network
        /// <returns></returns>
        public int GetPrefixBytes(int offset, byte[] serviceName, byte[] fullMessageSizeBytes, byte[] messagBuffer)
        {
            // Service Name bytes
            System.Buffer.BlockCopy(serviceName, 0, messagBuffer, 0, serviceName.Length);
            offset += serviceName.Length;

            // Full Message size bytes
            System.Buffer.BlockCopy(fullMessageSizeBytes, 0, messagBuffer, offset, fullMessageSizeBytes.Length);
            offset += fullMessageSizeBytes.Length;

            return offset;
        }

        /// <summary>
        ///    Serialize a ROS message in the expected format of
        ///     https://github.com/Unity-Technologies/Robotics-Tutorials/tree/master/catkin_ws/src/tcp_endpoint
        ///
        /// 	All messages are expected to come in the format of:
        /// 		first four bytes: int32 of the length of following string value
        /// 		next N bytes determined from previous four bytes: ROS topic name as a string
        /// 		next four bytes: int32 of the length of the remaining bytes for the ROS Message
        /// 		last N bytes determined from previous four bytes: ROS Message variables
        /// </summary>
        /// <param name="topicServiceName"></param> The ROS topic or service name that is receiving the messsage
        /// <param name="message"></param> The ROS message to send to a ROS publisher or service
        /// <returns> byte array with serialized ROS message in appropriate format</returns>
        public byte[] GetMessageBytes(string topicServiceName, Message message)
        {
            byte[] topicName = message.SerializeString(topicServiceName);
            byte[] bytesMsg = message.Serialize();
            byte[] fullMessageSizeBytes = BitConverter.GetBytes(bytesMsg.Length);

            byte[] messageBuffer = new byte[topicName.Length + fullMessageSizeBytes.Length + bytesMsg.Length];
            // Copy topic name and message size in bytes to message buffer
            int offset = GetPrefixBytes(0, topicName, fullMessageSizeBytes, messageBuffer);
            // ROS message bytes
            System.Buffer.BlockCopy(bytesMsg, 0, messageBuffer, offset, bytesMsg.Length);

            return messageBuffer;
        }

        struct SysCommand_Subscribe
        {
            public string topic;
            public string message_name;
        }

        struct SysCommand_Publish
        {
            public string topic;
            public string message_name;
        }

        void SendSysCommand(string command, object param)
        {
            Send(SYSCOMMAND_TOPIC_NAME, new RosUnitySysCommand(command, JsonUtility.ToJson(param)));
        }

        public async void Send(string rosTopicName, Message message)
        {
            TcpClient client = null;
            try
            {
                client = new TcpClient();
                await client.ConnectAsync(rosIPAddress, rosPort);

                NetworkStream networkStream = client.GetStream();
                networkStream.ReadTimeout = networkTimeout;

                WriteDataStaggered(networkStream, rosTopicName, message);
            }
            catch (NullReferenceException e)
            {
                Debug.LogError("TCPConnector.SendMessage Null Reference Exception: " + e);
            }
            catch (Exception e)
            {
                Debug.LogError("TCPConnector Exception: " + e);
            }
            finally
            {
                if (client != null && client.Connected)
                {
                    try
                    {
                        if (hudPanel != null) hudPanel.SetLastMessageSent(rosTopicName, message);
                        client.Close();
                    }
                    catch (Exception)
                    {
                        //Ignored.
                    }
                }
            }
        }

        /// <summary>
        ///    Serialize a ROS message in the expected format of
        ///     https://github.com/Unity-Technologies/Robotics-Tutorials/tree/master/catkin_ws/src/tcp_endpoint
        ///
        /// 	All messages are expected to come in the format of:
        /// 		first four bytes: int32 of the length of following string value
        /// 		next N bytes determined from previous four bytes: ROS topic name as a string
        /// 		next four bytes: int32 of the length of the remaining bytes for the ROS Message
        /// 		last N bytes determined from previous four bytes: ROS Message variables
        /// </summary>
        /// <param name="networkStream"></param> The network stream that is transmitting the messsage
        /// <param name="rosTopicName"></param> The ROS topic or service name that is receiving the messsage
        /// <param name="message"></param> The ROS message to send to a ROS publisher or service
        private void WriteDataStaggered(NetworkStream networkStream, string rosTopicName, Message message)
        {
            byte[] topicName = message.SerializeString(rosTopicName);
            List<byte[]> segments = message.SerializationStatements();
            int messageLength = segments.Select(s => s.Length).Sum();
            byte[] fullMessageSizeBytes = BitConverter.GetBytes(messageLength);

            networkStream.Write(topicName, 0, topicName.Length);
            networkStream.Write(fullMessageSizeBytes, 0, fullMessageSizeBytes.Length);
            foreach (byte[] segmentData in segments)
            {
                networkStream.Write(segmentData, 0, segmentData.Length);
            }
        }
    }
}