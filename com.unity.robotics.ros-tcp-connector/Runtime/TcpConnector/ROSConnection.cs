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
using RosMessageTypes.UnityInterfaces;

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

        public static string RosIPAddressPref
        {
            get => PlayerPrefs.GetString("ROS_IP", "127.0.0.1");
            set => PlayerPrefs.SetString("ROS_IP", value);
        }


        [SerializeField]
        [FormerlySerializedAs("hostPort")]
        [FormerlySerializedAs("rosPort")]
        int m_RosPort = 10000;
        public int RosPort { get => m_RosPort; set => m_RosPort = value; }
        public static int RosPortPref
        {
            get => PlayerPrefs.GetInt("ROS_TCP_PORT", 10000);
            set => PlayerPrefs.SetInt("ROS_TCP_PORT", value);
        }

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

        const string k_SysCommand_Log = "__log";
        const string k_SysCommand_Warning = "__warn";
        const string k_SysCommand_Error = "__error";
        const string k_SysCommand_ServiceRequest = "__request";
        const string k_SysCommand_ServiceResponse = "__response";
        const string k_SysCommand_Subscribe = "__subscribe";
        const string k_SysCommand_Publish = "__publish";
        const string k_SysCommand_RosService = "__ros_service";
        const string k_SysCommand_UnityService = "__unity_service";
        const string k_SysCommand_TopicList = "__topic_list";

        // GUI window variables
        internal HUDPanel m_HudPanel = null;

        ConcurrentQueue<List<byte[]>> m_OutgoingMessages = new ConcurrentQueue<List<byte[]>>();
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
            public Func<MessageDeserializer, Message> messageConstructor;
            public string rosMessageName;
            public List<Action<Message>> callbacks;
        }

        Dictionary<string, SubscriberCallback> m_Subscribers = new Dictionary<string, SubscriberCallback>();

        struct UnityServiceImplementation
        {
            public Func<MessageDeserializer, Message> messageConstructor;
            public string rosMessageName;
            public Func<Message, Message> callback;
        }

        Dictionary<string, UnityServiceImplementation> m_UnityServices = new Dictionary<string, UnityServiceImplementation>();
        Dictionary<string, string> m_Publishers = new Dictionary<string, string>();
        Dictionary<string, string> m_RosServices = new Dictionary<string, string>();
        MessageSerializer m_MessageSerializer = new MessageSerializer();
        MessageDeserializer m_MessageDeserializer = new MessageDeserializer();
        List<Action<string[]>> m_TopicsListCallbacks = new List<Action<string[]>>();
        List<Action<Dictionary<string, string>>> m_TopicsAndTypesListCallbacks = new List<Action<Dictionary<string, string>>>();

        public void Subscribe<T>(string topic, Action<T> callback) where T : Message
        {
            string rosMessageName = rosMessageName = MessageRegistry.GetRosMessageName<T>();
            AddSubscriberInternal<T>(topic, rosMessageName, callback);

            if (HasConnectionThread)
                SendSubscriberRegistration(topic, rosMessageName);
        }

        void AddSubscriberInternal<T>(string topic, string rosMessageName, Action<T> callback) where T : Message
        {
            SubscriberCallback subCallbacks;
            if (!m_Subscribers.TryGetValue(topic, out subCallbacks))
            {
                subCallbacks = new SubscriberCallback
                {
                    messageConstructor = MessageRegistry.GetConstructor<T>(),
                    rosMessageName = rosMessageName,
                    callbacks = new List<Action<Message>> { }
                };
                m_Subscribers.Add(topic, subCallbacks);
            }

            subCallbacks.callbacks.Add((Message msg) =>
            {
                callback((T)msg);
            });
        }

        public void ImplementService<T>(string topic, Func<T, Message> callback) where T : Message
        {
            string rosMessageName = rosMessageName = MessageRegistry.GetRosMessageName<T>();
            m_UnityServices[topic] = new UnityServiceImplementation
            {
                messageConstructor = MessageRegistry.GetConstructor<T>(),
                rosMessageName = rosMessageName,
                callback = (Message msg) => callback((T)msg)
            };

            if (HasConnectionThread)
                SendUnityServiceRegistration(topic, rosMessageName);
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
            m_MessageSerializer.Clear();
            m_MessageSerializer.SerializeMessage(serviceRequest);
            byte[] requestBytes = m_MessageSerializer.GetBytes();
            TaskPauser pauser = new TaskPauser();

            int srvID;
            lock (m_ServiceRequestLock)
            {
                srvID = m_NextSrvID++;
                m_ServicesWaiting.Add(srvID, pauser);
            }

            SendSysCommand(k_SysCommand_ServiceRequest, new SysCommand_Service { srv_id = srvID });
            Send(rosServiceName, serviceRequest);

            byte[] rawResponse = (byte[])await pauser.PauseUntilResumed();

            RESPONSE result = m_MessageDeserializer.DeserializeMessage<RESPONSE>(rawResponse);
            return result;
        }

        public void GetTopicList(Action<string[]> callback)
        {
            m_TopicsListCallbacks.Add(callback);
            SendSysCommand(k_SysCommand_TopicList, new SysCommand_TopicsRequest());
        }

        public void GetTopicAndTypeList(Action<Dictionary<string, string>> callback)
        {
            m_TopicsAndTypesListCallbacks.Add(callback);
            SendSysCommand(k_SysCommand_TopicList, new SysCommand_TopicsRequest());
        }

        [Obsolete("Calling Subscribe now implicitly registers a subscriber")]
        public void RegisterSubscriber(string topic, string rosMessageName)
        {
        }

        public void RegisterPublisher<T>(string topic) where T : Message
        {
            RegisterPublisher(topic, MessageRegistry.GetRosMessageName<T>());
        }

        public void RegisterPublisher(string topic, string rosMessageName)
        {
            m_Publishers[topic] = rosMessageName;
            if (HasConnectionThread)
                SendPublisherRegistration(topic, rosMessageName);
        }

        public void RegisterRosService<T>(string topic) where T : Message
        {
            RegisterRosService(topic, MessageRegistry.GetRosMessageName<T>());
        }

        public void RegisterRosService(string topic, string rosMessageName)
        {
            m_RosServices[topic] = rosMessageName;
            if (HasConnectionThread)
                SendRosServiceRegistration(topic, rosMessageName);
        }

        [Obsolete("Calling ImplementUnityService now implicitly registers it")]
        public void RegisterUnityService(string topic, string rosMessageName)
        {
        }


        void SendSubscriberRegistration(string topic, string rosMessageName)
        {
            SendSysCommand(k_SysCommand_Subscribe, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
        }

        void SendPublisherRegistration(string topic, string rosMessageName)
        {
            SendSysCommand(k_SysCommand_Publish, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
        }

        void SendRosServiceRegistration(string topic, string rosMessageName)
        {
            SendSysCommand(k_SysCommand_RosService, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName });
        }

        void SendUnityServiceRegistration(string topic, string rosMessageName)
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
                        _instance = Instantiate(prefab).GetComponent<ROSConnection>();
                    }
                    _instance.m_RosIPAddress = RosIPAddressPref;
                    _instance.RosPort = RosPortPref;
                }

                return _instance;
            }
        }

        void OnEnable()
        {
            if (_instance == null)
                _instance = this;
        }

        void Start()
        {
            InitializeHUD();

            if (ConnectOnStart)
            {
                Connect();
            }
        }

        public void Connect(string ipAddress, int port)
        {
            PlayerPrefs.SetString("ROS_IP", ipAddress);
            PlayerPrefs.SetInt("ROS_TCP_PORT", port);
            Connect();
        }

        public void Connect()
        {
            string ipAddress = PlayerPrefs.GetString("ROS_IP", "127.0.0.1");
            int port = PlayerPrefs.GetInt("ROS_TCP_PORT", 10000);

            if (!IPFormatIsCorrect(ipAddress))
                Debug.LogError("ROS IP address is not correct");

            if (m_HudPanel != null)
                m_HudPanel.host = $"{ipAddress}:{port}";

            m_ConnectionThreadCancellation = new CancellationTokenSource();

            Task.Run(() => ConnectionThread(ipAddress, port, m_NetworkTimeoutSeconds, m_KeepaliveTime, (int)(m_SleepTimeSeconds * 1000.0f), RegisterAll, m_OutgoingMessages, m_IncomingMessages, m_ConnectionThreadCancellation.Token));
        }

        void RegisterAll()
        {
            foreach (var keyValue in m_Subscribers)
            {
                if (keyValue.Value.rosMessageName != null)
                    SendSubscriberRegistration(keyValue.Key, keyValue.Value.rosMessageName);
            }

            foreach (var keyValue in m_UnityServices)
            {
                if (keyValue.Value.rosMessageName != null)
                    SendUnityServiceRegistration(keyValue.Key, keyValue.Value.rosMessageName);
            }

            foreach (var keyValue in m_Publishers)
            {
                if (keyValue.Value != null)
                    SendPublisherRegistration(keyValue.Key, keyValue.Value);
            }

            foreach (var keyValue in m_RosServices)
            {
                if (keyValue.Value != null)
                    SendRosServiceRegistration(keyValue.Key, keyValue.Value);
            }
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

        Action<string, byte[]> m_SpecialIncomingMessageHandler;

        void Update()
        {
            s_RealTimeSinceStartup = Time.realtimeSinceStartup;

            Tuple<string, byte[]> data;
            while (m_IncomingMessages.TryDequeue(out data))
            {
                (string topic, byte[] contents) = data;

                if (m_SpecialIncomingMessageHandler != null)
                {
                    m_SpecialIncomingMessageHandler(topic, contents);
                }
                else if (topic.StartsWith("__"))
                {
                    ReceiveSysCommand(topic, Encoding.UTF8.GetString(contents));
                }
                else
                {
                    // notify whatever is interested in this incoming message
                    SubscriberCallback callback;
                    if (m_Subscribers.TryGetValue(topic, out callback))
                    {
                        m_MessageDeserializer.InitWithBuffer(contents);
                        Message message = callback.messageConstructor(m_MessageDeserializer);

                        if (m_HudPanel != null && !topic.StartsWith("__"))
                            m_HudPanel.SetLastMessageReceived(topic, message);

                        callback.callbacks.ForEach(item => item(message));
                    }
                }
            }
        }

        void ReceiveSysCommand(string topic, string json)
        {
            switch (topic)
            {
                case k_SysCommand_Log:
                    {
                        var logCommand = JsonUtility.FromJson<SysCommand_Log>(json);
                        Debug.Log(logCommand.text);
                    }
                    break;
                case k_SysCommand_Warning:
                    {
                        var logCommand = JsonUtility.FromJson<SysCommand_Log>(json);
                        Debug.LogWarning(logCommand.text);
                    }
                    break;
                case k_SysCommand_Error:
                    {
                        var logCommand = JsonUtility.FromJson<SysCommand_Log>(json);
                        Debug.LogError(logCommand.text);
                    }
                    break;
                case k_SysCommand_ServiceRequest:
                    {
                        var serviceCommand = JsonUtility.FromJson<SysCommand_Service>(json);

                        // the next incoming message will be a service request, so set a special callback to process it
                        m_SpecialIncomingMessageHandler = (string serviceTopic, byte[] requestBytes) =>
                        {
                            m_SpecialIncomingMessageHandler = null;

                            // find the service implementation
                            UnityServiceImplementation service;
                            if (!m_UnityServices.TryGetValue(serviceTopic, out service))
                            {
                                Debug.LogError($"Unity service {serviceTopic} has not been implemented!");
                                return;
                            }

                            // deserialize the request message
                            m_MessageDeserializer.InitWithBuffer(requestBytes);
                            Message requestMessage = service.messageConstructor(m_MessageDeserializer);

                            // run the actual service
                            Message responseMessage = service.callback(requestMessage);

                            // send the response message back
                            SendSysCommand(k_SysCommand_ServiceResponse, new SysCommand_Service { srv_id = serviceCommand.srv_id });
                            Send(serviceTopic, responseMessage);
                        };
                    }
                    break;

                case k_SysCommand_ServiceResponse:
                    {
                        // it's a response from a ros service
                        var serviceCommand = JsonUtility.FromJson<SysCommand_Service>(json);
                        m_SpecialIncomingMessageHandler = (string serviceTopic, byte[] requestBytes) =>
                        {
                            m_SpecialIncomingMessageHandler = null;

                            TaskPauser resumer;
                            lock (m_ServiceRequestLock)
                            {
                                if (!m_ServicesWaiting.TryGetValue(serviceCommand.srv_id, out resumer))
                                {
                                    Debug.LogError($"Unable to route service response on \"{serviceTopic}\"! SrvID {serviceCommand.srv_id} does not exist.");
                                    return;
                                }

                                m_ServicesWaiting.Remove(serviceCommand.srv_id);
                            }
                            resumer.Resume(requestBytes);
                        };
                    }
                    break;

                case k_SysCommand_TopicList:
                    {
                        var topicsResponse = JsonUtility.FromJson<SysCommand_TopicsResponse>(json);
                        if (m_TopicsAndTypesListCallbacks.Count > 0)
                        {
                            Dictionary<string, string> callbackParam = new Dictionary<string, string>();
                            for (int Idx = 0; Idx < topicsResponse.topics.Length; ++Idx)
                                callbackParam[topicsResponse.topics[Idx]] = topicsResponse.types[Idx];
                            m_TopicsAndTypesListCallbacks.ForEach(a => a(callbackParam));
                            m_TopicsAndTypesListCallbacks.Clear();
                        }
                        if (m_TopicsListCallbacks.Count > 0)
                        {
                            m_TopicsListCallbacks.ForEach(a => a(topicsResponse.topics));
                            m_TopicsListCallbacks.Clear();
                        }
                    }
                    break;
            }
        }

        static void SendKeepalive(NetworkStream stream)
        {
            // 8 zeroes = a ros message with topic "" and no message data.
            stream.Write(new byte[] { 0, 0, 0, 0, 0, 0, 0, 0 }, 0, 8);
        }

        static void ClearMessageQueue(ConcurrentQueue<List<byte[]>> queue)
        {
            List<byte[]> unused;
            while (queue.TryDequeue(out unused))
            {
            }
        }

        static async Task ConnectionThread(
            string rosIPAddress,
            int rosPort,
            float networkTimeoutSeconds,
            float keepaliveTime,
            int sleepMilliseconds,
            Action RegisterAll,
            ConcurrentQueue<List<byte[]>> outgoingQueue,
            ConcurrentQueue<Tuple<string, byte[]>> incomingQueue,
            CancellationToken token)
        {
            //Debug.Log("ConnectionThread begins");
            int nextReaderIdx = 101;
            int nextReconnectionDelay = 1000;
            MessageSerializer messageSerializer = new MessageSerializer();

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

                    //ClearMessageQueue(outgoingQueue);
                    SendKeepalive(networkStream);

                    RegisterAll();

                    readerCancellation = new CancellationTokenSource();
                    _ = Task.Run(() => ReaderThread(nextReaderIdx, networkStream, incomingQueue, sleepMilliseconds, readerCancellation.Token));
                    nextReaderIdx++;

                    // connected, now just watch our queue for outgoing messages to send (or else send a keepalive message occasionally)
                    while (true)
                    {
                        List<byte[]> data;
                        float waitingSinceRealTime = s_RealTimeSinceStartup;
                        token.ThrowIfCancellationRequested();
                        while (!outgoingQueue.TryDequeue(out data))
                        {
                            // nothing to send right now, let's wait and see if something comes in
                            Thread.Sleep(sleepMilliseconds);
                            if (s_RealTimeSinceStartup > waitingSinceRealTime + keepaliveTime)
                            {
                                SendKeepalive(networkStream);
                                waitingSinceRealTime = s_RealTimeSinceStartup;
                            }
                            token.ThrowIfCancellationRequested();
                        }

                        foreach (byte[] statement in data)
                            networkStream.Write(statement, 0, statement.Length);
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
                    ClearMessageQueue(outgoingQueue);
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

        public struct SysCommand_TopicAndType
        {
            public string topic;
            public string message_name;
        }

        struct SysCommand_Log
        {
            public string text;
        }

        struct SysCommand_Service
        {
            public int srv_id;
        }

        struct SysCommand_TopicsRequest
        {
        }

        struct SysCommand_TopicsResponse
        {
            public string[] topics;
            public string[] types;
        }

        void SendSysCommand(string command, object param)
        {
            m_MessageSerializer.Clear();
            // syscommands are sent as:
            // 4 byte command length, followed by that many bytes of the command
            // (all command names start with __ to distinguish them from ros topics)
            m_MessageSerializer.Write(command);
            // 4-byte json length, followed by a json string of that length
            m_MessageSerializer.Write(JsonUtility.ToJson(param));

            m_OutgoingMessages.Enqueue(m_MessageSerializer.GetBytesSequence());
        }

        public void Send<T>(string rosTopicName, T message) where T : Message
        {
            if (!rosTopicName.StartsWith("__"))
            {
                m_Publishers[rosTopicName] = MessageRegistry.GetRosMessageName<T>();
                if (m_HudPanel != null)
                    m_HudPanel.SetLastMessageSent(rosTopicName, message);
            }

            m_MessageSerializer.Clear();
            // ros messages sent on our network channel contain:
            // 4 byte topic length, followed by that many bytes of the topic name
            m_MessageSerializer.Write(rosTopicName);
            // 4-byte message length, followed by that many bytes of the message
            m_MessageSerializer.SerializeMessageWithLength(message);

            m_OutgoingMessages.Enqueue(m_MessageSerializer.GetBytesSequence());
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
