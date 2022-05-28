using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using UnityEngine.Serialization;
using System.Collections.Concurrent;
using System.Threading;
using System.Linq;
using JetBrains.Annotations;

namespace Unity.Robotics.ROSTCPConnector
{
    public class ROSConnection : MonoBehaviour
    {
        public const string k_Version = "v0.7.1";
        public const string k_CompatibleVersionPrefix = "v0.7.";

        [SerializeField]
        bool m_ConnectOnStart = true;
        public bool ConnectOnStart { get => m_ConnectOnStart; set => m_ConnectOnStart = value; }

        [SerializeField]
        bool m_IsRos2 = false;
        public bool IsRos2 { get => m_IsRos2; set => m_IsRos2 = value; }

        [SerializeField]
        [FormerlySerializedAs("showHUD")]
        bool m_ShowHUD = true;
        public bool ShowHud { get => m_ShowHUD; set => m_ShowHUD = value; }

        [SerializeField]
        string[] m_TFTopics = { "/tf" };
        public string[] TFTopics { get => m_TFTopics; set => m_TFTopics = value; }

        const int k_DefaultPublisherQueueSize = 10;
        const bool k_DefaultPublisherLatch = false;

        // GUI window variables
        internal HudPanel m_HudPanel = null;
        public HudPanel HUDPanel => m_HudPanel;

        readonly object m_ServiceRequestLock = new object();

        int m_NextSrvID = 101;
        Dictionary<int, TaskPauser> m_ServicesWaiting = new Dictionary<int, TaskPauser>();

        public bool listenForTFMessages = true;

        public bool HasConnection => m_ConnectionTransport.HasConnection;
        public bool HasConnectionError => m_ConnectionTransport.HasConnectionError;

        public bool HasSubscriber(string topic)
        {
            RosTopicState info;
            return m_Topics.TryGetValue(topic, out info) && info.HasSubscriberCallback;
        }

        IConnectionTransport m_ConnectionTransport;
        public IConnectionTransport ConnectionTransport { get => m_ConnectionTransport; set => m_ConnectionTransport = value; }

        ISerializationProvider m_SerializationProvider;
        public ISerializationProvider SerializationProvider { get => m_SerializationProvider; set => m_SerializationProvider = value; }

        IMessageDeserializer m_MessageDeserializer;

        List<Action<string[]>> m_TopicsListCallbacks = new List<Action<string[]>>();
        List<Action<Dictionary<string, string>>> m_TopicsAndTypesListCallbacks = new List<Action<Dictionary<string, string>>>();
        List<Action<TimeSpan>> m_PingCallbacks = new List<Action<TimeSpan>>();
        List<Action<RosTopicState>> m_NewTopicCallbacks = new List<Action<RosTopicState>>();

        Dictionary<string, RosTopicState> m_Topics = new Dictionary<string, RosTopicState>();

        public void ListenForTopics(Action<RosTopicState> callback, bool notifyAllExistingTopics = false)
        {
            m_NewTopicCallbacks.Add(callback);
            if (notifyAllExistingTopics)
            {
                foreach (RosTopicState state in AllTopics)
                {
                    callback(state);
                }
            }
        }

        RosTopicState AddTopic(string topic, string rosMessageName, bool isService = false)
        {
            RosTopicState newTopic = new RosTopicState(topic, rosMessageName, this, new InternalAPI(this), isService);
            lock (m_Topics)
            {
                m_Topics.Add(topic, newTopic);
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

        public RosTopicState GetOrCreateTopic(string topic, string rosMessageName, bool isService = false)
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

            RosTopicState result = AddTopic(topic, rosMessageName, isService);
            foreach (Action<RosTopicState> callback in m_NewTopicCallbacks)
            {
                callback(result);
            }
            return result;
        }

        public void Subscribe<T>(string topic, Action<T> callback) where T : Message
        {
            string rosMessageName = MessageRegistry.GetRosMessageName<T>();
            AddSubscriberInternal(topic, rosMessageName, (Message msg) =>
            {
                if (msg.RosMessageName == rosMessageName)
                {
                    callback((T)msg);
                }
                else
                {
                    Debug.LogError($"Subscriber to '{topic}' expected '{rosMessageName}' but received '{msg.RosMessageName}'!?");
                }
            });
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
            var constructor = MessageRegistry.GetRosDeserializeFunction(rosMessageName);
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

            foreach (Action<RosTopicState> topicCallback in m_NewTopicCallbacks)
            {
                topicCallback(info);
            }
        }

        // Implement a service in Unity
        public void ImplementService<TRequest, TResponse>(string topic, Func<TRequest, TResponse> callback, int? queueSize = null)
            where TRequest : Message
            where TResponse : Message
        {
            string rosMessageName = rosMessageName = MessageRegistry.GetRosMessageName<TRequest>();

            RosTopicState info;
            if (!m_Topics.TryGetValue(topic, out info))
            {
                info = AddTopic(topic, rosMessageName, isService: true);
            }

            int resolvedQueueSize = queueSize.GetValueOrDefault(k_DefaultPublisherQueueSize);
            info.ImplementService(callback, resolvedQueueSize);

            foreach (Action<RosTopicState> topicCallback in m_NewTopicCallbacks)
            {
                topicCallback(info);
            }
        }

        // Implement a service in Unity
        public void ImplementService<TRequest, TResponse>(string topic, Func<TRequest, Task<TResponse>> callback, int? queueSize = null)
            where TRequest : Message
            where TResponse : Message
        {
            string rosMessageName = rosMessageName = MessageRegistry.GetRosMessageName<TRequest>();

            RosTopicState info;
            if (!m_Topics.TryGetValue(topic, out info))
            {
                info = AddTopic(topic, rosMessageName, isService: true);
            }

            int resolvedQueueSize = queueSize.GetValueOrDefault(k_DefaultPublisherQueueSize);
            info.ImplementService(callback, resolvedQueueSize);

            foreach (Action<RosTopicState> topicCallback in m_NewTopicCallbacks)
            {
                topicCallback(info);
            }
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
            TaskPauser pauser = new TaskPauser();

            int srvID;
            lock (m_ServiceRequestLock)
            {
                srvID = m_NextSrvID++;
                m_ServicesWaiting.Add(srvID, pauser);
            }

            RosTopicState topicState = GetOrCreateTopic(rosServiceName, serviceRequest.RosMessageName, isService: true);
            topicState.SendServiceRequest(serviceRequest, srvID);

            byte[] rawResponse = (byte[])await pauser.PauseUntilResumed();

            topicState.OnMessageReceived(rawResponse);
            RESPONSE result = m_MessageDeserializer.DeserializeMessage<RESPONSE>(rawResponse);
            return result;
        }

        public void GetTopicList(Action<string[]> callback)
        {
            m_TopicsListCallbacks.Add(callback);
            SendSysCommand(SysCommand.k_SysCommand_TopicList, new SysCommand_TopicsRequest());
        }

        public void GetTopicAndTypeList(Action<Dictionary<string, string>> callback)
        {
            m_TopicsAndTypesListCallbacks.Add(callback);
            SendSysCommand(SysCommand.k_SysCommand_TopicList, new SysCommand_TopicsRequest());
        }

        [Obsolete("Calling Subscribe now implicitly registers a subscriber")]
        public void RegisterSubscriber(string topic, string rosMessageName)
        {
        }

        public RosTopicState RegisterPublisher<T>(string rosTopicName,
            int? queue_size = null, bool? latch = null) where T : Message
        {
            return RegisterPublisher(rosTopicName, MessageRegistry.GetRosMessageName<T>(), queue_size, latch);
        }

        public RosTopicState RegisterPublisher(string rosTopicName, string rosMessageName,
            int? queueSize = null, bool? latch = null)
        {
            RosTopicState topicState = GetOrCreateTopic(rosTopicName, rosMessageName);
            //Create a new publisher.
            int resolvedQueueSize = queueSize.GetValueOrDefault(k_DefaultPublisherQueueSize);
            bool resolvedLatch = latch.GetValueOrDefault(k_DefaultPublisherLatch);
            topicState.RegisterPublisher(resolvedQueueSize, resolvedLatch);
            return topicState;
        }

        public void RegisterRosService<TRequest, TResponse>(string topic) where TRequest : Message where TResponse : Message
        {
            RegisterRosService(topic, MessageRegistry.GetRosMessageName<TRequest>(), MessageRegistry.GetRosMessageName<TResponse>());
        }

        public void RegisterRosService(string topic, string requestMessageName, string responseMessageName, int? queueSize = null)
        {
            RosTopicState info = GetOrCreateTopic(topic, requestMessageName, isService: true);
            int resolvedQueueSize = queueSize.GetValueOrDefault(k_DefaultPublisherQueueSize);
            info.RegisterRosService(responseMessageName, resolvedQueueSize);
        }

        [Obsolete("Calling ImplementUnityService now implicitly registers it")]
        public void RegisterUnityService(string topic, string rosMessageName)
        {
        }

        internal struct InternalAPI
        {
            ROSConnection m_Self;

            public InternalAPI(ROSConnection self) { m_Self = self; }

            public IMessageDeserializer Deserializer => m_Self.m_MessageDeserializer;

            public void SendSubscriberRegistration(string topic, string rosMessageName, IMessageSerializer serializer = null)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_Subscribe, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, serializer);
            }

            public void SendRosServiceRegistration(string topic, string rosMessageName, IMessageSerializer serializer = null)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_RosService, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, serializer);
            }

            public void SendUnityServiceRegistration(string topic, string rosMessageName, IMessageSerializer serializer = null)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_UnityService, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, serializer);
            }

            public void SendSubscriberUnregistration(string topic, IMessageSerializer serializer = null)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_RemoveSubscriber, new SysCommand_Topic { topic = topic }, serializer);
            }

            public void SendPublisherUnregistration(string topic, IMessageSerializer serializer = null)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_RemovePublisher, new SysCommand_Topic { topic = topic }, serializer);
            }

            public void SendRosServiceUnregistration(string topic, IMessageSerializer serializer = null)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_RemoveRosService, new SysCommand_Topic { topic = topic }, serializer);
            }

            public void SendUnityServiceUnregistration(string topic, IMessageSerializer serializer = null)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_RemoveUnityService, new SysCommand_Topic { topic = topic }, serializer);
            }

            public void SendUnityServiceResponse(int serviceId, IMessageSerializer serializer = null)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_ServiceResponse, new SysCommand_Service { srv_id = serviceId }, serializer);
            }

            public void SendPublisherRegistration(string topic, string message_name, int queueSize, bool latch, IMessageSerializer serializer = null)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_Publish,
                    new SysCommand_PublisherRegistration { topic = topic, message_name = message_name, queue_size = queueSize, latch = latch }
                );
            }

            public void SendServiceRequest(int serviceId)
            {
                m_Self.SendSysCommand(SysCommand.k_SysCommand_ServiceRequest, new SysCommand_Service { srv_id = serviceId });
            }

            public void AddSenderToQueue(ISendQueueItem sender)
            {
                m_Self.m_ConnectionTransport.Send(sender);
            }
        }

        static ROSConnection _instance;

        public static ROSConnection GetOrCreateInstance()
        {
            if (_instance == null)
            {
                // Prefer to use the ROSConnection in the scene, if any
                _instance = FindObjectOfType<ROSConnection>();
                if (_instance != null)
                    return _instance;

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

        void Awake()
        {
            if (_instance == null)
                _instance = this;
        }

        void Start()
        {
            InitializeHUD();

            if (listenForTFMessages)
                TFSystem.GetOrCreateInstance();

            m_SerializationProvider = new RosSerializationProvider(m_IsRos2);
            m_MessageDeserializer = m_SerializationProvider.CreateDeserializer();

            m_ConnectionTransport = GetComponent<IConnectionTransport>();
            if (m_ConnectionTransport == null)
            {
                Debug.LogError("Unable to find a ConnectionTransport!");
                return;
            }

            m_ConnectionTransport.Init(m_SerializationProvider, OnConnectionStartedCallback, OnConnectionLostCallback);

            if (ConnectOnStart)
                m_ConnectionTransport.Connect();
        }

        public void Connect()
        {
        }


        // NB this callback is not running on the main thread, be cautious about modifying data here
        void OnConnectionStartedCallback(IMessageSerializer serializer)
        {
            m_SpecialIncomingMessageHandler = HandshakeHandler;

            RosTopicState[] topics;
            lock (m_Topics)
            {
                topics = AllTopics.ToArray();
            }

            foreach (RosTopicState topicInfo in topics.ToArray())
                topicInfo.OnConnectionEstablished(serializer);

            RefreshTopicsList();
        }

        void HandshakeHandler(string name, byte[] payload)
        {
            // this is how we handle the first message on a new connection
            if (name == SysCommand.k_SysCommand_Handshake)
            {
                ReceiveSysCommand(name, Encoding.UTF8.GetString(payload));
            }
            else
            {
                Debug.LogError($"Invalid ROS-TCP-Endpoint version detected: 0.6.0 or older. Expected: {k_Version}.");
            }
        }

        void OnConnectionLostCallback()
        {
            RosTopicState[] topics;
            lock (m_Topics)
            {
                topics = AllTopics.ToArray();
            }

            foreach (RosTopicState topicInfo in topics)
            {
                //For all publishers, notify that they need to re-register.
                topicInfo.OnConnectionLost();
            }
        }

        Action<string, byte[]> m_SpecialIncomingMessageHandler;

        void Update()
        {
            if (m_ConnectionTransport.HasConnection &&
                m_TopicsRefreshRequested &&
                Time.realtimeSinceStartup - m_LastTopicsRequestSentRealtime > k_TimeBetweenTopicsUpdates)
            {
                SendTopicsRequest();
            }

            string topic;
            byte[] contents;
            while (m_ConnectionTransport.TryRead(out topic, out contents))
            {
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
                    {
                        try
                        {
                            //Add a try catch so that bad logic from one received message doesn't
                            //cause the Update method to exit without processing other received messages.
                            topicInfo.OnMessageReceived(contents);
                        }
                        catch (Exception e)
                        {
                            Debug.LogException(e);
                        }

                    }
                }
            }
        }

        bool m_TopicsRefreshRequested;
        float m_LastTopicsRequestSentRealtime = -1;
        const float k_TimeBetweenTopicsUpdates = 5.0f;

        public void RefreshTopicsList()
        {
            m_TopicsRefreshRequested = true;
        }

        void SendTopicsRequest()
        {
            m_LastTopicsRequestSentRealtime = Time.realtimeSinceStartup;
            m_TopicsRefreshRequested = false;

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
                case SysCommand.k_SysCommand_Handshake:
                    {
                        var handshakeCommand = JsonUtility.FromJson<SysCommand_Handshake>(json);
                        if (handshakeCommand.version == null)
                        {
                            Debug.LogError($"Corrupted or unreadable ROS-TCP-Endpoint version data! Expected: {k_Version}");
                        }
                        else if (!handshakeCommand.version.StartsWith(k_CompatibleVersionPrefix))
                        {
                            Debug.LogError($"Incompatible ROS-TCP-Endpoint version: {handshakeCommand.version}. Expected: {k_Version}");
                        }

                        var handshakeMetadata = JsonUtility.FromJson<SysCommand_Handshake_Metadata>(handshakeCommand.metadata);
#if ROS2
                        if (handshakeMetadata.protocol != "ROS2")
                        {
                            Debug.LogError($"Incompatible protocol: ROS-TCP-Endpoint is using {handshakeMetadata.protocol}, but Unity is in ROS2 mode. Switch it from the Robotics/Ros Settings menu.");
                        }
#else
                        if (handshakeMetadata.protocol != "ROS1")
                        {
                            Debug.LogError($"Incompatible protocol: ROS-TCP-Endpoint is using {handshakeMetadata.protocol}, but Unity is in ROS1 mode. Switch it from the Robotics/Ros Settings menu.");
                        }
#endif
                    }
                    break;
                case SysCommand.k_SysCommand_Log:
                    {
                        var logCommand = JsonUtility.FromJson<SysCommand_Log>(json);
                        Debug.Log(logCommand.text);
                    }
                    break;
                case SysCommand.k_SysCommand_Warning:
                    {
                        var logCommand = JsonUtility.FromJson<SysCommand_Log>(json);
                        Debug.LogWarning(logCommand.text);
                    }
                    break;
                case SysCommand.k_SysCommand_Error:
                    {
                        var logCommand = JsonUtility.FromJson<SysCommand_Log>(json);
                        Debug.LogError(logCommand.text);
                    }
                    break;
                case SysCommand.k_SysCommand_ServiceRequest:
                    {
                        var serviceCommand = JsonUtility.FromJson<SysCommand_Service>(json);

                        // the next incoming message will be a request for a Unity service, so set a special callback to process it
                        m_SpecialIncomingMessageHandler = (string serviceTopic, byte[] requestBytes) =>
                        {
                            m_SpecialIncomingMessageHandler = null;

                            RosTopicState topicState = GetTopic(serviceTopic);
                            if (topicState == null)
                            {
                                Debug.LogError($"Unity service {serviceTopic} has not been implemented!");
                                return;
                            }

                            topicState.HandleUnityServiceRequest(requestBytes, serviceCommand.srv_id);
                        };
                    }
                    break;

                case SysCommand.k_SysCommand_ServiceResponse:
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

                case SysCommand.k_SysCommand_TopicList:
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

                case SysCommand.k_SysCommand_PingResponse:
                    {
                        var pingResponse = JsonUtility.FromJson<SysCommand_PingResponse>(json);
                        if (m_PingCallbacks.Count > 0)
                        {
                            TimeSpan roundTripTime = DateTime.UtcNow - DateTime.Parse(pingResponse.request_time, null, System.Globalization.DateTimeStyles.RoundtripKind);
                            m_PingCallbacks.ForEach(a => a(roundTripTime));
                            m_PingCallbacks.Clear();
                        }
                    }
                    break;
            }
        }

        void OnApplicationQuit()
        {
            m_ConnectionTransport.Disconnect();
        }

        void SendSysCommand(string command, object param, IMessageSerializer serializer = null)
        {
            if (serializer != null)
                SendSysCommandImmediate(command, param, serializer);
            else
                QueueSysCommand(command, param);
        }

        public void Ping(Action<TimeSpan> callback)
        {
            m_PingCallbacks.Add(callback);

            string time8601 = DateTime.UtcNow.ToString("o", System.Globalization.CultureInfo.InvariantCulture);
            SendSysCommand("__ping", new SysCommand_PingRequest { request_time = time8601 });
        }

        static void SendSysCommandImmediate(string command, object param, [NotNull] IMessageSerializer messageSerializer)
        {
            messageSerializer.SendString(command, JsonUtility.ToJson(param));
        }

        public void QueueSysCommand(string command, object param)
        {
            m_ConnectionTransport.Send(command, JsonUtility.ToJson(param));
        }

        public void Publish(string rosTopicName, Message message)
        {
            if (rosTopicName.StartsWith("__"))
            {
                QueueSysCommand(rosTopicName, message);
            }
            else
            {
                RosTopicState rosTopic = GetTopic(rosTopicName);
                if (rosTopic == null || !rosTopic.IsPublisher)
                {
                    throw new Exception($"No registered publisher on topic {rosTopicName} for type {message.RosMessageName}!");
                }

                rosTopic.Publish(message);
            }
        }

        void InitializeHUD()
        {
            if (!Application.isPlaying || (!m_ShowHUD && m_HudPanel == null))
                return;

            if (m_HudPanel == null)
            {
                m_HudPanel = gameObject.AddComponent<HudPanel>();
            }

            m_HudPanel.isEnabled = m_ShowHUD;
        }
    }
}
