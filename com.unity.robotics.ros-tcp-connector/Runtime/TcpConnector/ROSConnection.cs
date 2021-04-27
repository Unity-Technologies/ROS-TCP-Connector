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
using System.Globalization;
using UnityEngine;
using UnityEngine.Serialization;
using System.Collections.Concurrent;
using System.Threading;

namespace Unity.Robotics.ROSTCPConnector
{
    public class ROSConnection : MonoBehaviour
    {
        // Variables required for ROS communication
        [FormerlySerializedAs("hostName")] public string rosIPAddress = "127.0.0.1";
        [FormerlySerializedAs("hostPort")] public int rosPort = 10000;

        private int networkTimeout = 2000;

        [Tooltip("Send keepalive message if nothing has been sent for this long (seconds).")]
        public float keepaliveTime = 10;

        const string ERROR_TOPIC_NAME = "__error";
        const string SYSCOMMAND_TOPIC_NAME = "__syscommand";
        const string HANDSHAKE_TOPIC_NAME = "__handshake";
        const string SERVICE_TOPIC_NAME = "__srv";

        const string SYSCOMMAND_SUBSCRIBE = "subscribe";
        const string SYSCOMMAND_PUBLISH = "publish";
        const string SYSCOMMAND_ROSSERVICE = "ros_service";
        const string SYSCOMMAND_UNITYSERVICE = "unity_service";

        // GUI window variables
        internal HUDPanel hudPanel = null;

        public bool showHUD = true;

        struct SubscriberCallback
        {
            public Func<Message> messageConstructor;
            public List<Action<Message>> callbacks;
        }

        Dictionary<string, SubscriberCallback> m_Subscribers = new Dictionary<string, SubscriberCallback>();

        struct UnityServiceImplementation
        {
            public Func<Message> messageConstructor;
            public Func<Message, Message> callback;
        }

        Dictionary<string, UnityServiceImplementation> m_UnityServices = new Dictionary<string, UnityServiceImplementation>();

        public void Subscribe<T>(string topic, Action<T> callback) where T : Message, new()
        {
            SubscriberCallback subCallbacks;
            if (!m_Subscribers.TryGetValue(topic, out subCallbacks))
            {
                subCallbacks = new SubscriberCallback
                {
                    messageConstructor = ()=> new T(),
                    callbacks = new List<Action<Message>> { }
                };
                m_Subscribers.Add(topic, subCallbacks);
            }

            subCallbacks.callbacks.Add((Message msg) =>
            {
                callback((T)msg);
            });
        }

        public void ImplementService<T>(string topic, Func<T, Message> callback)
            where T : Message, new()
        {
            m_UnityServices[topic] = new UnityServiceImplementation
            {
                messageConstructor = ()=> new T(),
                callback = (Message msg) => callback((T)msg)
            };
        }

        readonly object m_ServiceRequestLock = new object();
        int m_NextSrvID = 101;
        Dictionary<int, TaskPauser> m_ServiceCallbacks = new Dictionary<int, TaskPauser>();

        public async void SendServiceMessage<RESPONSE>(string rosServiceName, Message serviceRequest, Action<RESPONSE> callback) where RESPONSE : Message, new()
        {
            RESPONSE response = await SendServiceMessage<RESPONSE>(rosServiceName, serviceRequest);
            try
            {
                callback(response);
            }
            catch (Exception e)
            {
                Debug.LogError("Exception in service callback: " + e);
            }
        }

        public async Task<RESPONSE> SendServiceMessage<RESPONSE>(string rosServiceName, Message serviceRequest) where RESPONSE : Message, new()
        {
            byte[] requestBytes = serviceRequest.Serialize();
            TaskPauser pauser = new TaskPauser();

            int srvID;
            lock (m_ServiceRequestLock)
            {
                srvID = m_NextSrvID++;
                m_ServiceCallbacks.Add(srvID, pauser);
            }

            MRosUnitySrvMessage srvMessage = new MRosUnitySrvMessage(srvID, true, rosServiceName, requestBytes);
            Send(SERVICE_TOPIC_NAME, srvMessage);

            byte[] rawResponse = (byte[])await pauser.PauseUntilResumed();
            RESPONSE result = new RESPONSE();
            result.Deserialize(rawResponse, 0);
            return result;
        }

        public void GetTopicList(Action<string[]> callback)
        {
            SendServiceMessage<MRosUnityTopicListResponse>("__topic_list", new MRosUnityTopicListRequest(), response => callback(response.topics));
        }

        public void RegisterSubscriber(string topic, string rosMessageName)
        {
            SendSysCommand(SYSCOMMAND_SUBSCRIBE, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
        }

        public void RegisterPublisher(string topic, string rosMessageName)
        {
            SendSysCommand(SYSCOMMAND_PUBLISH, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
        }

        public void RegisterRosService(string topic, string rosMessageName)
        {
            SendSysCommand(SYSCOMMAND_ROSSERVICE, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
        }

        public void RegisterUnityService(string topic, string rosMessageName)
        {
            SendSysCommand(SYSCOMMAND_UNITYSERVICE, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
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

        ConcurrentQueue<Tuple<string, Message>> outgoingMessages = new ConcurrentQueue<Tuple<string, Message>>();
        ConcurrentQueue<Tuple<string, byte[]>> incomingMessages = new ConcurrentQueue<Tuple<string, byte[]>>();

        private void Start()
        {
            if (!IPFormatIsCorrect(rosIPAddress))
                Debug.LogError("ROS IP address is not correct");
            InitializeHUD();
            Subscribe<MRosUnityError>(ERROR_TOPIC_NAME, RosUnityErrorCallback);
            Subscribe<MRosUnitySrvMessage>(SERVICE_TOPIC_NAME, ProcessServiceMessage);

            connectionThreadCancellation = new CancellationTokenSource();
            Task.Run(() => ConnectionThread(rosIPAddress, rosPort, networkTimeout, keepaliveTime, outgoingMessages, incomingMessages, connectionThreadCancellation.Token));
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

        void RosUnityErrorCallback(MRosUnityError error)
        {
            Debug.LogError("ROS-Unity error: " + error.message);
        }

        private void Update()
        {
            s_RealTimeSinceStartup = Time.realtimeSinceStartup;

            Tuple<string, byte[]> data;
            while (incomingMessages.TryDequeue(out data))
            {
                (string topic, byte[] contents) = data;

                // notify whatever is interested in this incoming message
                SubscriberCallback callback;
                if (m_Subscribers.TryGetValue(topic, out callback))
                {
                    Message message = callback.messageConstructor();
                    message.Deserialize(contents, 0);
                    callback.callbacks.ForEach(item => item(message));
                }
            }
        }

        void ProcessServiceMessage(MRosUnitySrvMessage message)
        {
            if (message.is_request)
            {
                UnityServiceImplementation service;
                if(m_UnityServices.TryGetValue(message.topic, out service))
                {
                    Message requestMessage = service.messageConstructor();
                    requestMessage.Deserialize(message.payload, 0);
                    Message responseMessage = service.callback(requestMessage);
                    byte[] responseBytes = responseMessage.Serialize();
                    Send(SERVICE_TOPIC_NAME, new MRosUnitySrvMessage(message.srv_id, false, message.topic, responseBytes));
                }
            }
            else
            {
                TaskPauser resumer;
                lock (m_ServiceRequestLock)
                {
                    if (!m_ServiceCallbacks.TryGetValue(message.srv_id, out resumer))
                    {
                        Debug.LogError($"Unable to route service response on \"{message.topic}\"! SrvID {message.srv_id} does not exist.");
                        return;
                    }

                    m_ServiceCallbacks.Remove(message.srv_id);
                }
                resumer.Resume(message.payload);
            }
        }

        CancellationTokenSource connectionThreadCancellation;
        static bool m_ShouldConnect;
        static float s_RealTimeSinceStartup = 0.0f;// only the main thread can access Time.realTimeSinceStartup, so make a copy here

        static void SendKeepalive(NetworkStream stream)
        {
            // 8 zeroes = a ros message with topic "" and no message data.
            stream.Write(new byte[] { 0, 0, 0, 0, 0, 0, 0, 0 }, 0, 8);
        }

        static async Task ConnectionThread(
            string rosIPAddress,
            int rosPort,
            int networkTimeout,
            float keepaliveTime,
            ConcurrentQueue<Tuple<string, Message>> outgoingQueue,
            ConcurrentQueue<Tuple<string, byte[]>> incomingQueue,
            CancellationToken token)
        {
            Debug.Log("ConnectionThread begins");
            int nextReaderIdx = 101;
            int nextReconnectionDelay = 1000;

            while (!token.IsCancellationRequested)
            {
                TcpClient client = null;
                CancellationTokenSource readerCancellation = null;

                try
                {
                    client = new TcpClient();
                    client.Connect(rosIPAddress, rosPort);

                    NetworkStream networkStream = client.GetStream();
                    networkStream.ReadTimeout = networkTimeout;

                    SendKeepalive(networkStream);

                    readerCancellation = new CancellationTokenSource();
                    _ = Task.Run(() => ReaderThread(nextReaderIdx, networkStream, incomingQueue, readerCancellation.Token));
                    nextReaderIdx++;

                    // connected, now just watch our queue for outgoing messages to send (or else send a keepalive message occasionally)
                    while (true)
                    {
                        Tuple<string, Message> data;
                        float waitingSinceRealTime = s_RealTimeSinceStartup;
                        token.ThrowIfCancellationRequested();
                        while (!outgoingQueue.TryDequeue(out data))
                        {
                            Thread.Yield();
                            if (s_RealTimeSinceStartup > waitingSinceRealTime + keepaliveTime)
                            {
                                SendKeepalive(networkStream);
                                waitingSinceRealTime = s_RealTimeSinceStartup;
                            }
                            token.ThrowIfCancellationRequested();
                        }

                        WriteDataStaggered(networkStream, data.Item1, data.Item2);
                    }
                }
                catch (OperationCanceledException)
                {
                }
                catch (Exception e)
                {
                    Debug.Log($"Connection to {rosIPAddress}:{rosPort} failed - " + e);
                    await Task.Delay(nextReconnectionDelay);
                }
                finally
                {
                    if (readerCancellation != null)
                        readerCancellation.Cancel();

                    if (client != null)
                        client.Close();

                    // clear the message queue
                    Tuple<string, Message> unused;
                    while (outgoingQueue.TryDequeue(out unused))
                    {
                    }
                }
                await Task.Yield();
            }
        }

        static async Task ReaderThread(int readerIdx, NetworkStream networkStream, ConcurrentQueue<Tuple<string, byte[]>> queue, CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                try
                {
                    Tuple<string, byte[]> content = await ReadMessageContents(networkStream, token);
                    //Debug.Log($"Message {content.Item1} received");
                    queue.Enqueue(content);
                }
                catch (OperationCanceledException)
                {
                }
                catch (Exception e)
                {
                    Debug.Log("Reader "+readerIdx+" exception! " + e);
                }
            }
        }

        static async Task ReadToByteArray(NetworkStream networkStream, byte[] array, int length, CancellationToken token)
        {
            int read = 0;
            while (read < length && networkStream.CanRead)
            {
                while (!token.IsCancellationRequested && !networkStream.DataAvailable)
                    await Task.Yield();

                token.ThrowIfCancellationRequested();
                read += await networkStream.ReadAsync(array, read, length - read, token);
            }

            if (read < length)
                throw new SocketException(); // the connection has closed
        }

        static byte[] fourBytes = new byte[4];
        static byte[] topicScratchSpace = new byte[64];

        static async Task<Tuple<string, byte[]>> ReadMessageContents(NetworkStream networkStream, CancellationToken token)
        {
            // Get first bytes to determine length of topic name
            await ReadToByteArray(networkStream, fourBytes, 4, token);
            int topicLength = BitConverter.ToInt32(fourBytes, 0);

            // If our topic buffer isn't large enough, make a larger one (and keep it that size; assume that's the new standard)
            if (topicLength > topicScratchSpace.Length)
                topicScratchSpace = new byte[topicLength];

            // Read and convert topic name
            await ReadToByteArray(networkStream, topicScratchSpace, topicLength, token);
            string topicName = Encoding.ASCII.GetString(topicScratchSpace, 0, topicLength);

            await ReadToByteArray(networkStream, fourBytes, 4, token);
            int full_message_size = BitConverter.ToInt32(fourBytes, 0);

            byte[] readBuffer = new byte[full_message_size];
            await ReadToByteArray(networkStream, readBuffer, full_message_size, token);

            return Tuple.Create(topicName, readBuffer);
        }

        void OnApplicationQuit()
        {
            connectionThreadCancellation.Cancel();
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

        struct SysCommand_TopicAndType
        {
            public string topic;
            public string message_name;
        }

        void SendSysCommand(string command, object param)
        {
            Send(SYSCOMMAND_TOPIC_NAME, new MRosUnitySysCommand(command, JsonUtility.ToJson(param)));
        }

        public void Send(string rosTopicName, Message message)
        {
            outgoingMessages.Enqueue(new Tuple<string, Message>(rosTopicName, message));
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
        static void WriteDataStaggered(NetworkStream networkStream, string rosTopicName, Message message)
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

        public static bool IPFormatIsCorrect(string ipAddress)
        {
            if(ipAddress == null || ipAddress == "")
                return false;
            
            // If IP address is set using static lookup tables https://man7.org/linux/man-pages/man5/hosts.5.html
            if(Char.IsLetter(ipAddress[0]))
            {
                foreach(Char subChar in ipAddress)
                {
                    if(!(Char.IsLetterOrDigit(subChar)  || subChar == '-'|| subChar == '.'))
                        return false;
                }

                if(!Char.IsLetterOrDigit(ipAddress[ipAddress.Length - 1]))
                    return false;
                return true;
            }

            string[] subAdds = ipAddress.Split('.');
            if(subAdds.Length != 4)
            {
                return false;
            }
            IPAddress parsedipAddress;
            return IPAddress.TryParse(ipAddress, out parsedipAddress);
        }
    }
}