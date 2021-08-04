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
using RosMessageTypes.BuiltinInterfaces;

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

        private const int k_DefaultPublisherQueueSize = 10;
        private const bool k_DefaultPublisherLatch = false;

        // GUI window variables
        internal HUDPanel m_HudPanel = null;

        class OutgoingMessageQueue
        {
            private ConcurrentQueue<SendsOutgoingMessages> outgoingMessageQueue;
            public readonly ManualResetEvent NewMessageReadyToSendEvent;

            public OutgoingMessageQueue()
            {
                outgoingMessageQueue = new ConcurrentQueue<SendsOutgoingMessages>();;
                NewMessageReadyToSendEvent = new ManualResetEvent(false);
            }

            public void Enqueue(SendsOutgoingMessages sendsOutgoingMessages)
            {
                outgoingMessageQueue.Enqueue(sendsOutgoingMessages);
                NewMessageReadyToSendEvent.Set();
            }

            public bool TryDequeue(out SendsOutgoingMessages sendsOutgoingMessages)
            {
                return outgoingMessageQueue.TryDequeue(out sendsOutgoingMessages);
            }
        }

        private OutgoingMessageQueue m_OutgoingMessageQueue = new OutgoingMessageQueue();

        ConcurrentQueue<Tuple<string, byte[]>> m_IncomingMessages = new ConcurrentQueue<Tuple<string, byte[]>>();
        CancellationTokenSource m_ConnectionThreadCancellation;

        public bool HasConnectionThread => m_ConnectionThreadCancellation != null;

        static bool m_HasConnectionError = false;
        public bool HasConnectionError => m_HasConnectionError;

        // only the main thread can access Time.*, so make a copy here
        public static float s_RealTimeSinceStartup = 0.0f;
        private static float s_SimulatedTimeSinceStartup = 0.0f;

        readonly object m_ServiceRequestLock = new object();
        int m_NextSrvID = 101;
        Dictionary<int, TaskPauser> m_ServicesWaiting = new Dictionary<int, TaskPauser>();

        struct SubscriberCallback
        {
            public Func<MessageDeserializer, Message> deserialize;
            public string rosMessageName;
            public List<Action<Message>> callbacks;
        }

        Dictionary<string, SubscriberCallback> m_Subscribers = new Dictionary<string, SubscriberCallback>();

        struct UnityServiceImplementation
        {
            public Func<MessageDeserializer, Message> deserialize;
            public string rosMessageName;
            public Func<Message, Message> callback;
        }

        private object dictionaryLock = new object();
        Dictionary<string, UnityServiceImplementation> m_UnityServices = new Dictionary<string, UnityServiceImplementation>();
        Dictionary<string, ROSPublisherBase> m_Publishers = new Dictionary<string, ROSPublisherBase>();
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

        public static TimeMsg SimulationTime
        {
            get
            {
                var simTimeSeconds = (uint) Mathf.FloorToInt(s_SimulatedTimeSinceStartup);
                var simTimeNanoSeconds =
                    (uint) ((s_SimulatedTimeSinceStartup - (double) simTimeSeconds) * 1000000000.0);
                return new TimeMsg(simTimeSeconds, simTimeNanoSeconds);
            }
        }

        void AddSubscriberInternal<T>(string topic, string rosMessageName, Action<T> callback) where T : Message
        {
            SubscriberCallback subCallbacks;
            if (!m_Subscribers.TryGetValue(topic, out subCallbacks))
            {
                subCallbacks = new SubscriberCallback
                {
                    deserialize = MessageRegistry.GetDeserializeFunction<T>(),
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

        public void ImplementService<REQUEST>(string topic, Func<REQUEST, Message> callback) where REQUEST : Message
        {
            string rosMessageName = rosMessageName = MessageRegistry.GetRosMessageName<REQUEST>();
            m_UnityServices[topic] = new UnityServiceImplementation
            {
                deserialize = MessageRegistry.GetDeserializeFunction<REQUEST>(),
                rosMessageName = rosMessageName,
                callback = (Message msg) => callback((REQUEST)msg)
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

            SendSysCommand(SysCommand.k_SysCommand_ServiceRequest, new SysCommand_Service { srv_id = srvID });
            Publish(rosServiceName, serviceRequest);

            byte[] rawResponse = (byte[])await pauser.PauseUntilResumed();

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

        public ROSPublisher<T> GetOrRegisterPublisher<T>(string rosTopicName,
            int? queue_size = null, bool? latch = null) where T : Message
        {
            ROSPublisherBase publisher;
            int resolvedQueueSize = queue_size.GetValueOrDefault(k_DefaultPublisherQueueSize);
            bool resolvedLatch = latch.GetValueOrDefault(k_DefaultPublisherLatch);
            lock(dictionaryLock)
            {
                if (m_Publishers.TryGetValue(rosTopicName, out publisher))
                {
                    if (publisher.EquivalentTo(rosTopicName, typeof(T), queue_size, latch))
                    {
                        //We already have a valid existing publisher of the correct type.
                    }
                    else
                    {
                        Debug.LogWarning($"Publisher on topic {rosTopicName} has changed type! " +
                                         $"Do you have multiple publishers on the same topic?");
                        publisher = new ROSPublisher<T>(rosTopicName, resolvedQueueSize, resolvedLatch);
                        m_Publishers[rosTopicName] = publisher;
                    }
                }
                else
                {
                    //Create a new publisher.
                    publisher = new ROSPublisher<T>(rosTopicName, resolvedQueueSize, resolvedLatch);
                    m_Publishers[rosTopicName] = publisher;
                }
            }

            ROSPublisher<T> existingPublisher = (ROSPublisher<T>) publisher;
            if (existingPublisher == null)
            {
                //Note this shouldn't happen, but to remove compiler warnings a null check is added.
                throw new InvalidCastException("Failed publisher cast!");
            }

            return existingPublisher;
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


        void SendSubscriberRegistration(string topic, string rosMessageName, NetworkStream stream = null)
        {
            SendSysCommand(SysCommand.k_SysCommand_Subscribe, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, stream);
        }

        void SendRosServiceRegistration(string topic, string rosMessageName, NetworkStream stream = null)
        {
            SendSysCommand(SysCommand.k_SysCommand_RosService, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, stream);
        }

        void SendUnityServiceRegistration(string topic, string rosMessageName, NetworkStream stream = null)
        {
            SendSysCommand(SysCommand.k_SysCommand_UnityService, new SysCommand_TopicAndType { topic = topic, message_name = rosMessageName }, stream);
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
            RosIPAddress = ipAddress;
            RosPort = port;
            Connect();
        }

        public void Connect()
        {
            if (!IPFormatIsCorrect(RosIPAddress))
                Debug.LogWarning("Invalid ROS IP address: " + RosIPAddress);

            if (m_HudPanel != null)
                m_HudPanel.host = $"{RosIPAddress}:{RosPort}";

            m_ConnectionThreadCancellation = new CancellationTokenSource();

            Task.Run(() => ConnectionThread(
                RosIPAddress,
                RosPort,
                m_NetworkTimeoutSeconds,
                m_KeepaliveTime,
                (int)(m_SleepTimeSeconds * 1000.0f),
                RegisterAll,
                DeregisterAll,
                m_OutgoingMessageQueue,
                m_IncomingMessages,
                m_ConnectionThreadCancellation.Token
            ));
        }

        void RegisterAll(NetworkStream stream)
        {
            lock (dictionaryLock)
            {
                foreach (var keyValue in m_Subscribers)
                {
                    if (keyValue.Value.rosMessageName != null)
                        SendSubscriberRegistration(keyValue.Key, keyValue.Value.rosMessageName, stream);
                }

                foreach (var keyValue in m_UnityServices)
                {
                    if (keyValue.Value.rosMessageName != null)
                        SendUnityServiceRegistration(keyValue.Key, keyValue.Value.rosMessageName, stream);
                }

                foreach (var keyValue in m_Publishers)
                {
                    if (keyValue.Value != null)
                        keyValue.Value.OnConnectionEstablished(m_MessageSerializer, stream);
                }

                foreach (var keyValue in m_RosServices)
                {
                    if (keyValue.Value != null)
                        SendRosServiceRegistration(keyValue.Key, keyValue.Value, stream);
                }
            }
        }

        void DeregisterAll()
        {
            lock(dictionaryLock)
            {
                foreach (var keyValue in m_Publishers)
                {
                    //For all publishers, notify that they need to re-register.
                    if (keyValue.Value != null)
                        keyValue.Value.PublisherRegistered = false;
                }
            }
        }

        public void Disconnect()
        {
            m_ConnectionThreadCancellation?.Cancel();
            //The thread may be waiting on a ManualResetEvent, if so, this will wake it so it can exit immediately.
            m_OutgoingMessageQueue?.NewMessageReadyToSendEvent?.Set();
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

        private void FixedUpdate()
        {
            s_RealTimeSinceStartup = Time.realtimeSinceStartup;
            s_SimulatedTimeSinceStartup = Time.time;
        }

        void Update()
        {
            s_RealTimeSinceStartup = Time.realtimeSinceStartup;
            s_SimulatedTimeSinceStartup = Time.time;

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
                        Message message = callback.deserialize(m_MessageDeserializer);

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
                            Message requestMessage = service.deserialize(m_MessageDeserializer);

                            // run the actual service
                            Message responseMessage = service.callback(requestMessage);

                            // send the response message back
                            SendSysCommand(SysCommand.k_SysCommand_ServiceResponse, new SysCommand_Service { srv_id = serviceCommand.srv_id });
                            Publish(serviceTopic, responseMessage);
                        };
                    }
                    break;

                case SysCommand.k_SysCommand_ServiceResponse:
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

                case SysCommand.k_SysCommand_TopicList:
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

        static void ClearMessageQueue(OutgoingMessageQueue queue)
        {
            while (queue.TryDequeue(out SendsOutgoingMessages sendsOutgoingMessages))
            {
                sendsOutgoingMessages.ClearAllQueuedData();
            }
        }

        static async Task ConnectionThread(
            string rosIPAddress,
            int rosPort,
            float networkTimeoutSeconds,
            float keepaliveTime,
            int sleepMilliseconds,
            Action<NetworkStream> RegisterAll,
            Action DeregisterAll,
            OutgoingMessageQueue outgoingQueue,
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
                    RegisterAll(networkStream);

                    readerCancellation = new CancellationTokenSource();
                    _ = Task.Run(() => ReaderThread(nextReaderIdx, networkStream, incomingQueue, sleepMilliseconds, readerCancellation.Token));
                    nextReaderIdx++;

                    // connected, now just watch our queue for outgoing messages to send (or else send a keepalive message occasionally)
                    float waitingSinceRealTime = s_RealTimeSinceStartup;
                    while (true)
                    {

                        bool messagesToSend = outgoingQueue.NewMessageReadyToSendEvent.WaitOne(sleepMilliseconds);
                        token.ThrowIfCancellationRequested();

                        if (messagesToSend)
                        {
                            outgoingQueue.NewMessageReadyToSendEvent.Reset();
                        }
                        else
                        {
                            if (s_RealTimeSinceStartup > waitingSinceRealTime + keepaliveTime)
                            {
                                SendKeepalive(networkStream);
                                waitingSinceRealTime = s_RealTimeSinceStartup;
                            }
                        }

                        while (outgoingQueue.TryDequeue(out SendsOutgoingMessages sendsOutgoingMessages))
                        {

                            SendsOutgoingMessages.SendToState sendToState = sendsOutgoingMessages.SendInternal(messageSerializer, networkStream);
                            switch (sendToState)
                            {
                                case SendsOutgoingMessages.SendToState.Normal:
                                    //This is normal operation.
                                    break;
                                case SendsOutgoingMessages.SendToState.QueueFullWarning:
                                    //Unable to send messages to ROS as fast as we're generating them.
                                    //This could be caused by a TCP connection that is too slow.
                                    Debug.LogWarning($"Queue full! Messages are getting dropped! " +
                                                     "Try check your connection speed is fast enough to handle the traffic.");
                                    break;
                                case SendsOutgoingMessages.SendToState.NoMessageToSendError:
                                    //This indicates
                                    Debug.LogError(
                                        "Logic Error! A 'SendsOutgoingMessages' was queued but did not have any messages to send.");
                                    break;
                            }

                            token.ThrowIfCancellationRequested();
                            waitingSinceRealTime = s_RealTimeSinceStartup;
                        }
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
                    DeregisterAll();
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



        void SendSysCommand(string command, object param, NetworkStream stream = null)
        {
            if (stream != null)
                SendSysCommandImmediate(command, param, stream);
            else
                QueueSysCommand(command, param);
        }

        private static void PopulateSysCommand(MessageSerializer messageSerializer, string command, object param)
        {
            messageSerializer.Clear();
            // syscommands are sent as:
            // 4 byte command length, followed by that many bytes of the command
            // (all command names start with __ to distinguish them from ros topics)
            messageSerializer.Write(command);
            // 4-byte json length, followed by a json string of that length
            string json = JsonUtility.ToJson(param);
            messageSerializer.WriteUnaligned(json);
        }

        static void SendSysCommandImmediate(string command, object param, NetworkStream stream)
        {
            if (stream == null)
            {
                throw new ArgumentException("stream cannot be null!");
            }
            MessageSerializer messageSerializer = new MessageSerializer();
            PopulateSysCommand(messageSerializer, command, param);
            messageSerializer.SendTo(stream);
        }

        public void QueueSysCommand(string command, object param)
        {
            PopulateSysCommand(m_MessageSerializer, command, param);
            m_OutgoingMessageQueue.Enqueue(new SimpleDataSender(m_MessageSerializer.GetBytesSequence()));
        }

        public void Send<T>(string rosTopicName, T message, int? queue_size = null,
            bool? latch = null) where T : Message
        {
            Publish(rosTopicName, message, queue_size, latch);
        }

        public void Publish<T>(string rosTopicName, T message, int? queue_size = null,
            bool? latch = null) where T : Message
        {
            if (rosTopicName.StartsWith("__"))
            {
                QueueSysCommand(rosTopicName, message);
            }
            else
            {
                //TODO - It seems that the new system requires the publisher to be registered first, should this implementation be changed?
                //IE: if (!m_Publishers.ContainsKey(rosTopicName)) error unregistered topic!
                //Find the publisher and queue the message for sending.
                ROSPublisher<T> existingPublisher = GetOrRegisterPublisher<T>(rosTopicName, queue_size, latch);
                existingPublisher.Publish(message);
                m_OutgoingMessageQueue.Enqueue(existingPublisher);

                if (m_HudPanel != null)
                    m_HudPanel.SetLastMessageSent(rosTopicName, message);
            }

        }

        public static T GetFromPool<T>(string rosTopicName) where T : Message
        {
            ROSPublisher<T> rosPublisher = instance.GetOrRegisterPublisher<T>(rosTopicName);
            return rosPublisher.GetMessageFromPool();
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
