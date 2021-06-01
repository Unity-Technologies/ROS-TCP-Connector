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
        [SerializeField]
        [FormerlySerializedAs("hostName")]
        [FormerlySerializedAs("rosIPAddress")]
        string m_RosIPAddress = "127.0.0.1";
        public string RosIPAddress { get => m_RosIPAddress; set => m_RosIPAddress = value; }

        [SerializeField]
        [FormerlySerializedAs("hostPort")]
        [FormerlySerializedAs("rosPort")]
        int m_RosPort = 10000;
        public int RosPort { get => m_RosPort; set => m_RosPort = value; }

        [SerializeField]
        bool m_ConnectOnStart = true;
        public bool ConnectOnStart { get => m_ConnectOnStart; set => m_ConnectOnStart = value; }

        [SerializeField]
        [Tooltip("Send keepalive message if nothing has been sent for this long (seconds).")]
        float m_KeepaliveTime = 10;
        public float KeepaliveTime { get => m_KeepaliveTime; set => m_KeepaliveTime = value; }

        [SerializeField]
        float m_NetworkTimeoutSeconds = 2;
        public float NetworkTimeoutSeconds { get => m_NetworkTimeoutSeconds; set => m_NetworkTimeoutSeconds = value; }

        [SerializeField]
        float m_SleepTimeSeconds = 0.01f;
        public float SleepTimeSeconds { get => m_SleepTimeSeconds; set => m_SleepTimeSeconds = value; }

        [SerializeField]
        [FormerlySerializedAs("showHUD")]
        bool m_ShowHUD = true;
        public bool ShowHud { get => m_ShowHUD; set => m_ShowHUD = value; }

        const string k_Topic_Error = "__error";
        const string k_Topic_SysCommand = "__syscommand";
        const string k_Topic_Services = "__srv";

        const string k_SysCommand_Subscribe = "subscribe";
        const string k_SysCommand_Publish = "publish";
        const string k_SysCommand_RosService = "ros_service";
        const string k_SysCommand_UnityService = "unity_service";

        // GUI window variables
        internal HUDPanel m_HudPanel = null;

        ConcurrentQueue<Tuple<string, Message>> m_OutgoingMessages = new ConcurrentQueue<Tuple<string, Message>>();
        ConcurrentQueue<Tuple<string, byte[]>> m_IncomingMessages = new ConcurrentQueue<Tuple<string, byte[]>>();
        CancellationTokenSource m_ConnectionThreadCancellation;

        public bool HasConnectionThread => m_ConnectionThreadCancellation != null;

        static bool m_HasConnectionError = false;
        public bool HasConnectionError => m_HasConnectionError;

        static float s_RealTimeSinceStartup = 0.0f;// only the main thread can access Time.realTimeSinceStartup, so make a copy here

        readonly object m_ServiceRequestLock = new object();
        int m_NextSrvID = 101;
        Dictionary<int, TaskPauser> m_ServicesWaiting = new Dictionary<int, TaskPauser>();

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
                    messageConstructor = () => new T(),
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
                messageConstructor = () => new T(),
                callback = (Message msg) => callback((T)msg)
            };
        }

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
                m_ServicesWaiting.Add(srvID, pauser);
            }

            MRosUnitySrvMessage srvMessage = new MRosUnitySrvMessage(srvID, true, rosServiceName, requestBytes);
            Send(k_Topic_Services, srvMessage);

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
            SendSysCommand(k_SysCommand_Subscribe, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
        }

        public void RegisterPublisher(string topic, string rosMessageName)
        {
            SendSysCommand(k_SysCommand_Publish, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
        }

        public void RegisterRosService(string topic, string rosMessageName)
        {
            SendSysCommand(k_SysCommand_RosService, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
        }

        public void RegisterUnityService(string topic, string rosMessageName)
        {
            SendSysCommand(k_SysCommand_UnityService, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
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
            Subscribe<MRosUnityError>(k_Topic_Error, RosUnityErrorCallback);
            Subscribe<MRosUnitySrvMessage>(k_Topic_Services, ProcessServiceMessage);

            if (ConnectOnStart)
            {
                Connect();
            }
        }

        public void Connect(string ipAddress, int port)
        {
            m_RosIPAddress = ipAddress;
            m_RosPort = port;
            Connect();
        }

        public void Connect()
        {
            if (!IPFormatIsCorrect(m_RosIPAddress))
                Debug.LogError("ROS IP address is not correct");

            if (m_HudPanel != null)
                m_HudPanel.host = $"{m_RosIPAddress}:{m_RosPort}";

            m_ConnectionThreadCancellation = new CancellationTokenSource();
            Task.Run(() => ConnectionThread(m_RosIPAddress, m_RosPort, m_NetworkTimeoutSeconds, m_KeepaliveTime, (int)(m_SleepTimeSeconds * 1000.0f), m_OutgoingMessages, m_IncomingMessages, m_ConnectionThreadCancellation.Token));
        }

        public void Disconnect()
        {
            if (m_ConnectionThreadCancellation != null)
                m_ConnectionThreadCancellation.Cancel();
            m_ConnectionThreadCancellation = null;
        }

        void OnValidate()
        {
            InitializeHUD();
        }

        private void InitializeHUD()
        {
            if (!Application.isPlaying || (!m_ShowHUD && m_HudPanel == null))
                return;

            if (m_HudPanel == null)
            {
                m_HudPanel = gameObject.AddComponent<HUDPanel>();
                m_HudPanel.host = $"{RosIPAddress}:{RosPort}";
            }

            m_HudPanel.isEnabled = m_ShowHUD;
        }

        void RosUnityErrorCallback(MRosUnityError error)
        {
            Debug.LogError("ROS-Unity error: " + error.message);
        }

        private void Update()
        {
            s_RealTimeSinceStartup = Time.realtimeSinceStartup;

            Tuple<string, byte[]> data;
            while (m_IncomingMessages.TryDequeue(out data))
            {
                (string topic, byte[] contents) = data;

                // notify whatever is interested in this incoming message
                SubscriberCallback callback;
                if (m_Subscribers.TryGetValue(topic, out callback))
                {
                    Message message = callback.messageConstructor();
                    message.Deserialize(contents, 0);

                    if (m_HudPanel != null)
                        m_HudPanel.SetLastMessageReceived(topic, message);

                    callback.callbacks.ForEach(item => item(message));
                }
            }
        }

        void ProcessServiceMessage(MRosUnitySrvMessage message)
        {
            if (message.is_request)
            {
                UnityServiceImplementation service;
                if (m_UnityServices.TryGetValue(message.topic, out service))
                {
                    Message requestMessage = service.messageConstructor();
                    requestMessage.Deserialize(message.payload, 0);
                    Message responseMessage = service.callback(requestMessage);
                    byte[] responseBytes = responseMessage.Serialize();
                    Send(k_Topic_Services, new MRosUnitySrvMessage(message.srv_id, false, message.topic, responseBytes));
                }
            }
            else
            {
                TaskPauser resumer;
                lock (m_ServiceRequestLock)
                {
                    if (!m_ServicesWaiting.TryGetValue(message.srv_id, out resumer))
                    {
                        Debug.LogError($"Unable to route service response on \"{message.topic}\"! SrvID {message.srv_id} does not exist.");
                        return;
                    }

                    m_ServicesWaiting.Remove(message.srv_id);
                }
                resumer.Resume(message.payload);
            }
        }

        static void SendKeepalive(NetworkStream stream)
        {
            // 8 zeroes = a ros message with topic "" and no message data.
            stream.Write(new byte[] { 0, 0, 0, 0, 0, 0, 0, 0 }, 0, 8);
        }

        static async Task ConnectionThread(
            string rosIPAddress,
            int rosPort,
            float networkTimeoutSeconds,
            float keepaliveTime,
            int sleepMilliseconds,
            ConcurrentQueue<Tuple<string, Message>> outgoingQueue,
            ConcurrentQueue<Tuple<string, byte[]>> incomingQueue,
            CancellationToken token)
        {
            //Debug.Log("ConnectionThread begins");
            int nextReaderIdx = 101;
            int nextReconnectionDelay = 1000;

            while (!token.IsCancellationRequested)
            {
                TcpClient client = null;
                CancellationTokenSource readerCancellation = null;

                try
                {
                    ROSConnection.m_HasConnectionError = true; // until we actually see a reply back, assume there's a problem

                    client = new TcpClient();
                    client.Connect(rosIPAddress, rosPort);

                    NetworkStream networkStream = client.GetStream();
                    networkStream.ReadTimeout = (int)(networkTimeoutSeconds * 1000);

                    SendKeepalive(networkStream);

                    readerCancellation = new CancellationTokenSource();
                    _ = Task.Run(() => ReaderThread(nextReaderIdx, networkStream, incomingQueue, sleepMilliseconds, readerCancellation.Token));
                    nextReaderIdx++;

                    // connected, now just watch our queue for outgoing messages to send (or else send a keepalive message occasionally)
                    while (true)
                    {
                        Tuple<string, Message> data;
                        float waitingSinceRealTime = s_RealTimeSinceStartup;
                        token.ThrowIfCancellationRequested();
                        while (!outgoingQueue.TryDequeue(out data))
                        {
                            Thread.Sleep(sleepMilliseconds);
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
                    ROSConnection.m_HasConnectionError = true;
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

        static async Task ReaderThread(int readerIdx, NetworkStream networkStream, ConcurrentQueue<Tuple<string, byte[]>> queue, int sleepMilliseconds, CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                try
                {
                    Tuple<string, byte[]> content = await ReadMessageContents(networkStream, sleepMilliseconds, token);
                    //Debug.Log($"Message {content.Item1} received");
                    ROSConnection.m_HasConnectionError = false;

                    if (content.Item1 != "") // ignore keepalive messages
                        queue.Enqueue(content);
                }
                catch (OperationCanceledException)
                {
                }
                catch (Exception e)
                {
                    ROSConnection.m_HasConnectionError = true;
                    Debug.Log("Reader " + readerIdx + " exception! " + e);
                }
            }
        }

        static async Task ReadToByteArray(NetworkStream networkStream, byte[] array, int length, int sleepMilliseconds, CancellationToken token)
        {
            int read = 0;
            while (read < length && networkStream.CanRead)
            {
                while (!token.IsCancellationRequested && !networkStream.DataAvailable)
                    await Task.Delay(sleepMilliseconds);

                token.ThrowIfCancellationRequested();
                read += await networkStream.ReadAsync(array, read, length - read, token);
            }

            if (read < length)
                throw new SocketException(); // the connection has closed
        }

        static byte[] s_FourBytes = new byte[4];
        static byte[] s_TopicScratchSpace = new byte[64];

        static async Task<Tuple<string, byte[]>> ReadMessageContents(NetworkStream networkStream, int sleepMilliseconds, CancellationToken token)
        {
            // Get first bytes to determine length of topic name
            await ReadToByteArray(networkStream, s_FourBytes, 4, sleepMilliseconds, token);
            int topicLength = BitConverter.ToInt32(s_FourBytes, 0);

            // If our topic buffer isn't large enough, make a larger one (and keep it that size; assume that's the new standard)
            if (topicLength > s_TopicScratchSpace.Length)
                s_TopicScratchSpace = new byte[topicLength];

            // Read and convert topic name
            await ReadToByteArray(networkStream, s_TopicScratchSpace, topicLength, sleepMilliseconds, token);
            string topicName = Encoding.ASCII.GetString(s_TopicScratchSpace, 0, topicLength);

            await ReadToByteArray(networkStream, s_FourBytes, 4, sleepMilliseconds, token);
            int full_message_size = BitConverter.ToInt32(s_FourBytes, 0);

            byte[] readBuffer = new byte[full_message_size];
            await ReadToByteArray(networkStream, readBuffer, full_message_size, sleepMilliseconds, token);

            return Tuple.Create(topicName, readBuffer);
        }

        void OnApplicationQuit()
        {
            Disconnect();
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
            Send(k_Topic_SysCommand, new MRosUnitySysCommand(command, JsonUtility.ToJson(param)));
        }

        public void Send(string rosTopicName, Message message)
        {
            m_OutgoingMessages.Enqueue(new Tuple<string, Message>(rosTopicName, message));

            if (m_HudPanel != null)
                m_HudPanel.SetLastMessageSent(rosTopicName, message);
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
            if (ipAddress == null || ipAddress == "")
                return false;

            // If IP address is set using static lookup tables https://man7.org/linux/man-pages/man5/hosts.5.html
            if (Char.IsLetter(ipAddress[0]))
            {
                foreach (Char subChar in ipAddress)
                {
                    if (!(Char.IsLetterOrDigit(subChar) || subChar == '-' || subChar == '.'))
                        return false;
                }

                if (!Char.IsLetterOrDigit(ipAddress[ipAddress.Length - 1]))
                    return false;
                return true;
            }

            string[] subAdds = ipAddress.Split('.');
            if (subAdds.Length != 4)
            {
                return false;
            }
            IPAddress parsedipAddress;
            return IPAddress.TryParse(ipAddress, out parsedipAddress);
        }
    }
}
