using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
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
        [Tooltip("If nothing has been sent for this long (seconds), send a keepalive message to check the connection is still working.")]
        float m_KeepaliveTime = 1;
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

        [SerializeField]
        string[] m_TFTopics = { "/tf" };
        public string[] TFTopics { get => m_TFTopics; set => m_TFTopics = value; }

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
        const string k_SysCommand_RemoveSubscriber = "__remove_subscriber";
        const string k_SysCommand_RemovePublisher = "__remove_publisher";
        const string k_SysCommand_RemoveRosService = "__remove_ros_service";
        const string k_SysCommand_RemoveUnityService = "__remove_unity_service";

        // GUI window variables
        internal HudPanel m_HudPanel = null;
        public HudPanel HUDPanel => m_HudPanel;

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

        public bool listenForTFMessages = true;

        float m_LastMessageReceivedRealtime;
        float m_LastMessageSentRealtime;
        public float LastMessageReceivedRealtime => m_LastMessageReceivedRealtime;
        public float LastMessageSentRealtime => m_LastMessageSentRealtime;

        // For the IP address field we show in the hud, we store the IP address and port in PlayerPrefs.
        // This is used to remember the last IP address the player typed into the HUD, in builds where ConnectOnStart is not checked
        public const string PlayerPrefsKey_ROS_IP = "ROS_IP";
        public const string PlayerPrefsKey_ROS_TCP_PORT = "ROS_TCP_PORT";
        public static string RosIPAddressPref => PlayerPrefs.GetString(PlayerPrefsKey_ROS_IP, "127.0.0.1");
        public static int RosPortPref => PlayerPrefs.GetInt(PlayerPrefsKey_ROS_TCP_PORT, 10000);

        public bool HasSubscriber(string topic)
        {
            RosTopicState info;
            return m_Topics.TryGetValue(topic, out info) && info.HasSubscriberCallback;
        }

        MessageSerializer m_MessageSerializer = new MessageSerializer();
        MessageDeserializer m_MessageDeserializer = new MessageDeserializer();
        List<Action<string[]>> m_TopicsListCallbacks = new List<Action<string[]>>();
        List<Action<Dictionary<string, string>>> m_TopicsAndTypesListCallbacks = new List<Action<Dictionary<string, string>>>();
        List<Action<RosTopicState>> m_NewTopicCallbacks = new List<Action<RosTopicState>>();

        Dictionary<string, RosTopicState> m_Topics = new Dictionary<string, RosTopicState>();

        public void ListenForTopics(Action<RosTopicState> callback, bool notifyAllExistingTopics = false)
        {
            m_NewTopicCallbacks.Add(callback);
            if (notifyAllExistingTopics)
            {
                foreach (RosTopicState state in m_Topics.Values)
                {
                    callback(state);
                }
            }
        }

        public RosTopicState AddTopic(string topic, string rosMessageName)
        {
            RosTopicState newTopic = new RosTopicState(topic, rosMessageName, this, new InternalAPI(this));
            m_Topics.Add(topic, newTopic);
            foreach (Action<RosTopicState> callback in m_NewTopicCallbacks)
            {
                callback(newTopic);
            }
            return newTopic;
        }

        public RosTopicState GetTopic(string topic)
        {
            RosTopicState info;
            m_Topics.TryGetValue(topic, out info);
            return info;
        }

        public IEnumerable<RosTopicState> AllTopics => m_Topics.Values;

        public RosTopicState GetOrCreateTopic(string topic, string rosMessageName)
        {
            RosTopicState state = GetTopic(topic);
            if (state != null)
            {
                if (state.RosMessageName != rosMessageName)
                {
                    state.ChangeRosMessageName(rosMessageName);
                }
                return state;
            }

            return AddTopic(topic, rosMessageName);
        }

        public void Subscribe<T>(string topic, Action<T> callback) where T : Message
        {
            string rosMessageName = MessageRegistry.GetRosMessageName<T>();
            AddSubscriberInternal(topic, rosMessageName, (Message msg) => { callback((T)msg); });
        }

        public void Unsubscribe(string topic)
        {
            RosTopicState info = GetTopic(topic);
            if (info != null)
                info.UnsubscribeAll();
        }

        // Version for when the message type is unknown at compile time
        public void SubscribeByMessageName(string topic, string rosMessageName, Action<Message> callback)
        {
            var constructor = MessageRegistry.GetDeserializeFunction(rosMessageName);
            if (constructor == null)
            {
                Debug.LogError($"Failed to subscribe to topic {topic} - no class has RosMessageName \"{rosMessageName}\"!");
                return;
            }

            AddSubscriberInternal(topic, rosMessageName, callback);
        }

        void AddSubscriberInternal(string topic, string rosMessageName, Action<Message> callback)
        {
            RosTopicState info;
            if (!m_Topics.TryGetValue(topic, out info))
            {
                info = AddTopic(topic, rosMessageName);
            }

            info.AddSubscriber(callback);
        }

        // Implement a service in Unity
        public void ImplementService<TRequest>(string topic, Func<TRequest, Message> callback)
            where TRequest : Message
        {
            string rosMessageName = rosMessageName = MessageRegistry.GetRosMessageName<TRequest>();

            RosTopicState info;
            if (!m_Topics.TryGetValue(topic, out info))
            {
                info = AddTopic(topic, rosMessageName);
            }

            info.ImplementService((Message msg) => callback((TRequest)msg));
        }

        // Send a request to a ros service
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

        // Send a request to a ros service
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
            SendInternal(rosServiceName, serviceRequest);

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
            RosTopicState info = GetOrCreateTopic(topic, rosMessageName);
            info.RegisterPublisher();
        }

        public void RegisterRosService<TRequest, TResponse>(string topic) where TRequest : Message where TResponse : Message
        {
            RegisterRosService(topic, MessageRegistry.GetRosMessageName<TRequest>(), MessageRegistry.GetRosMessageName<TResponse>());
        }

        public void RegisterRosService(string topic, string requestMessageName, string responseMessageName)
        {
            RosTopicState info = GetOrCreateTopic(topic, requestMessageName);
            info.RegisterRosService(responseMessageName);
        }

        [Obsolete("Calling ImplementUnityService now implicitly registers it")]
        public void RegisterUnityService(string topic, string rosMessageName)
        {
        }

        internal struct InternalAPI
        {
            ROSConnection m_Self;

            public InternalAPI(ROSConnection self) { m_Self = self; }

            public MessageDeserializer Deserializer => m_Self.m_MessageDeserializer;

            public void SendSubscriberRegistration(string topic, string rosMessageName, NetworkStream stream = null)
            {
                m_Self.SendSysCommand(k_SysCommand_Subscribe, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, stream);
            }

            public void SendPublisherRegistration(string topic, string rosMessageName, NetworkStream stream = null)
            {
                m_Self.SendSysCommand(k_SysCommand_Publish, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, stream);
            }

            public void SendRosServiceRegistration(string topic, string rosMessageName, NetworkStream stream = null)
            {
                m_Self.SendSysCommand(k_SysCommand_RosService, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, stream);
            }

            public void SendUnityServiceRegistration(string topic, string rosMessageName, NetworkStream stream = null)
            {
                m_Self.SendSysCommand(k_SysCommand_UnityService, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, stream);
            }

            public void SendSubscriberUnregistration(string topic, NetworkStream stream = null)
            {
                m_Self.SendSysCommand(k_SysCommand_RemoveSubscriber, new SysCommand_Topic { topic = topic }, stream);
            }

            public void SendPublisherUnregistration(string topic, NetworkStream stream = null)
            {
                m_Self.SendSysCommand(k_SysCommand_RemovePublisher, new SysCommand_Topic { topic = topic }, stream);
            }

            public void SendRosServiceUnregistration(string topic, NetworkStream stream = null)
            {
                m_Self.SendSysCommand(k_SysCommand_RemoveRosService, new SysCommand_Topic { topic = topic }, stream);
            }

            public void SendUnityServiceUnregistration(string topic, NetworkStream stream = null)
            {
                m_Self.SendSysCommand(k_SysCommand_RemoveUnityService, new SysCommand_Topic { topic = topic }, stream);
            }
        }

        private static ROSConnection _instance;

        public static ROSConnection GetOrCreateInstance()
        {
            if (_instance == null)
            {
                GameObject prefab = Resources.Load<GameObject>("ROSConnectionPrefab");
                if (prefab == null)
                {
                    Debug.LogWarning("No settings for ROSConnection.instance! Open \"ROS Settings\" from the Robotics menu to configure it.");
                    GameObject instance = new GameObject("ROSConnection");
                    _instance = instance.AddComponent<ROSConnection>();
                }
                else
                {
                    _instance = Instantiate(prefab).GetComponent<ROSConnection>();
                }
            }
            return _instance;
        }

        [Obsolete("Please call ROSConnection.GetOrCreateInstance()")]
        public static ROSConnection instance
        {
            get
            {
                return GetOrCreateInstance();
            }
        }

        private void Awake()
        {
            if (_instance == null)
                _instance = this;
        }

        void Start()
        {
            InitializeHUD();

            HudPanel.RegisterHeader(DrawHeaderGUI);

            if (listenForTFMessages)
                TFSystem.GetOrCreateInstance(m_TFTopics);

            if (ConnectOnStart)
                Connect();
        }

        public void Connect(string ipAddress, int port)
        {
            RosIPAddress = ipAddress;
            RosPort = port;
            Connect();
        }

        public void Connect()
        {
            if (!IPFormatIsCorrect(RosIPAddress))
                Debug.LogWarning("Invalid ROS IP address: " + RosIPAddress);

            m_ConnectionThreadCancellation = new CancellationTokenSource();

            Task.Run(() => ConnectionThread(
                RosIPAddress,
                RosPort,
                m_NetworkTimeoutSeconds,
                m_KeepaliveTime,
                (int)(m_SleepTimeSeconds * 1000.0f),
                OnConnectionStartedCallback,
                m_OutgoingMessages,
                m_IncomingMessages,
                m_ConnectionThreadCancellation.Token
            ));
        }

        // NB this callback is not running on the main thread, be cautious about modifying data here
        void OnConnectionStartedCallback(NetworkStream stream)
        {
            foreach (RosTopicState topicInfo in AllTopics)
                topicInfo.RegisterAll(stream);

            RefreshTopicsList();
        }

        public void Disconnect()
        {
            if (m_ConnectionThreadCancellation != null)
                m_ConnectionThreadCancellation.Cancel();
            m_ConnectionThreadCancellation = null;
        }

        void OnValidate()
        {
            // the prefab is not the instance!
            if (gameObject.scene.name == null)
                return;

            if (_instance == null)
                _instance = this;
        }

        Action<string, byte[]> m_SpecialIncomingMessageHandler;

        void Update()
        {
            s_RealTimeSinceStartup = Time.realtimeSinceStartup;

            Tuple<string, byte[]> data;
            while (m_IncomingMessages.TryDequeue(out data))
            {
                (string topic, byte[] contents) = data;
                m_LastMessageReceivedRealtime = Time.realtimeSinceStartup;

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
                    RosTopicState topicInfo = GetTopic(topic);
                    // if this is null, we have received a message on a topic we've never heard of...!?
                    // all we can do is ignore it, we don't even know what type it is
                    if (topicInfo != null)
                        topicInfo.OnMessageReceived(contents);
                }
            }
        }

        float m_LastTopicsRequestRealtime = -1;
        const float k_TimeBetweenTopicsUpdates = 5.0f;

        public void RefreshTopicsList()
        {
            // cap the rate of requests
            if (m_LastTopicsRequestRealtime != -1 && s_RealTimeSinceStartup - m_LastTopicsRequestRealtime <= k_TimeBetweenTopicsUpdates)
                return;

            m_LastTopicsRequestRealtime = s_RealTimeSinceStartup;
            GetTopicAndTypeList((data) =>
            {
                foreach (KeyValuePair<string, string> kv in data)
                {
                    RosTopicState state = GetOrCreateTopic(kv.Key, kv.Value);
                    if (state.RosMessageName != kv.Value)
                    {
                        state.ChangeRosMessageName(kv.Value);
                    }
                }
            });
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

                        // the next incoming message will be a request for a Unity service, so set a special callback to process it
                        m_SpecialIncomingMessageHandler = (string serviceTopic, byte[] requestBytes) =>
                        {
                            m_SpecialIncomingMessageHandler = null;

                            RosTopicState topicState = GetTopic(serviceTopic);
                            if (topicState == null || !topicState.IsUnityService)
                            {
                                Debug.LogError($"Unity service {serviceTopic} has not been implemented!");
                                return;
                            }
                            Message responseMessage = topicState.HandleUnityServiceRequest(requestBytes);

                            // send the response message back
                            SendSysCommand(k_SysCommand_ServiceResponse, new SysCommand_Service { srv_id = serviceCommand.srv_id });
                            SendInternal(serviceTopic, responseMessage);
                        };
                    }
                    break;

                case k_SysCommand_ServiceResponse:
                    {
                        // the next incoming message will be a response from a ros service
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
                            for (int idx = 0; idx < topicsResponse.topics.Length; ++idx)
                                callbackParam[topicsResponse.topics[idx]] = topicsResponse.types[idx];
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
            Action<NetworkStream> OnConnectionStartedCallback,
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

                    SendKeepalive(networkStream);
                    OnConnectionStartedCallback(networkStream);

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
                    // Debug.Log($"Message {content.Item1} received");
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

        public struct SysCommand_Topic
        {
            public string topic;
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

        void SendSysCommand(string command, object param, NetworkStream stream = null)
        {
            m_MessageSerializer.Clear();
            // syscommands are sent as:
            // 4 byte command length, followed by that many bytes of the command
            // (all command names start with __ to distinguish them from ros topics)
            m_MessageSerializer.Write(command);
            // 4-byte json length, followed by a json string of that length
            string json = JsonUtility.ToJson(param);
            m_MessageSerializer.WriteUnaligned(json);

            if (stream != null)
                m_MessageSerializer.SendTo(stream);
            else
                m_OutgoingMessages.Enqueue(m_MessageSerializer.GetBytesSequence());
        }

        public void Send<T>(string rosTopicName, T message) where T : Message
        {
            Publish(rosTopicName, message);
        }

        public void Publish<T>(string rosTopicName, T message) where T : Message
        {
            if (!rosTopicName.StartsWith("__"))
            {
                RosTopicState rosTopic = GetTopic(rosTopicName);
                if (!rosTopic.IsPublisher)
                    Debug.LogError($"Can't publish a message to an unregistered topic '{rosTopicName}'");

                m_LastMessageReceivedRealtime = Time.realtimeSinceStartup;
                rosTopic.OnMessageSent(message);
            }
            SendInternal(rosTopicName, message);
        }

        void SendInternal<T>(string rosTopicName, T message) where T : Message
        {
            m_MessageSerializer.Clear();
            // ros messages sent on our network channel contain:
            // 4 byte topic length, followed by that many bytes of the topic name
            m_MessageSerializer.Write(rosTopicName);
            // 4-byte message length, followed by that many bytes of the message
            m_MessageSerializer.SerializeMessageWithLength(message);

            m_OutgoingMessages.Enqueue(m_MessageSerializer.GetBytesSequence());
        }

        private void InitializeHUD()
        {
            if (!Application.isPlaying || (!m_ShowHUD && m_HudPanel == null))
                return;

            if (m_HudPanel == null)
            {
                m_HudPanel = gameObject.AddComponent<HudPanel>();
            }

            m_HudPanel.isEnabled = m_ShowHUD;
        }

        void DrawHeaderGUI()
        {
            GUIStyle connectionArrowStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
                fontSize = 22,
                fontStyle = FontStyle.Bold,
                fixedWidth = 250
            };

            GUIStyle labelStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
                fontStyle = FontStyle.Bold,
                fixedWidth = 250
            };

            GUIStyle contentStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 0, 5),
                normal = { textColor = Color.white },
                fixedWidth = 300
            };


            // ROS IP Setup
            GUILayout.BeginHorizontal();
            var baseColor = GUI.color;
            GUI.color = Color.white;
            GUI.Label(new Rect(4, 5, 25, 15), "I", connectionArrowStyle);
            GUI.color = GetConnectionColor(Time.realtimeSinceStartup - LastMessageReceivedRealtime);
            GUI.Label(new Rect(8, 6, 25, 15), "\u2190", connectionArrowStyle);
            GUI.color = GetConnectionColor(Time.realtimeSinceStartup - LastMessageSentRealtime);
            GUI.Label(new Rect(8, 0, 25, 15), "\u2192", connectionArrowStyle);
            GUI.color = baseColor;

#if ROS2
            string protocolName = "ROS2";
#else
            string protocolName = "ROS";
#endif

            GUILayout.Space(30);
            GUILayout.Label($"{protocolName} IP: ", labelStyle, GUILayout.Width(100));

            if (!HasConnectionThread)
            {
                // if you've never run a build on this machine before, initialize the playerpref settings to the ones from the RosConnection
                if (!PlayerPrefs.HasKey(PlayerPrefsKey_ROS_IP))
                    SetIPPref(RosIPAddress);
                if (!PlayerPrefs.HasKey(PlayerPrefsKey_ROS_TCP_PORT))
                    SetPortPref(RosPort);

                // NB, here the user is editing the PlayerPrefs values, not the ones in the RosConnection.
                // (So that the hud remembers what IP you used last time you ran this build.)
                // The RosConnection receives the edited values when you click Connect.
                SetIPPref(GUILayout.TextField(RosIPAddressPref));
                SetPortPref(Convert.ToInt32(GUILayout.TextField(RosPortPref.ToString())));

                GUILayout.EndHorizontal();
                GUILayout.Label("(Not connected)");
                if (GUILayout.Button("Connect"))
                    Connect(RosIPAddressPref, RosPortPref);
            }
            else
            {
                GUILayout.Label($"{RosIPAddress}:{RosPort}", contentStyle);
                GUILayout.EndHorizontal();
            }
        }

        public static void SetIPPref(string ipAddress)
        {
            PlayerPrefs.SetString(PlayerPrefsKey_ROS_IP, ipAddress);
        }

        public static void SetPortPref(int port)
        {
            PlayerPrefs.SetInt(PlayerPrefsKey_ROS_TCP_PORT, port);
        }

        Color GetConnectionColor(float elapsedTime)
        {
            var bright = new Color(1, 1, 0.5f);
            var mid = new Color(0, 1, 1);
            var dark = new Color(0, 0.5f, 1);
            const float brightDuration = 0.03f;
            const float fadeToDarkDuration = 1.0f;

            if (!HasConnectionThread)
                return Color.gray;
            if (HasConnectionError)
                return Color.red;

            if (elapsedTime <= brightDuration)
                return bright;
            return Color.Lerp(mid, dark, elapsedTime / fadeToDarkDuration);
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
