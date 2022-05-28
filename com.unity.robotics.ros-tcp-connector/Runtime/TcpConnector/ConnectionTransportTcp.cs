using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class ConnectionTransportTcp : MonoBehaviour, IConnectionTransport
    {
        // Variables required for ROS communication
        [SerializeField]
        string m_RosIPAddress = "127.0.0.1";
        public string RosIPAddress { get => m_RosIPAddress; set => m_RosIPAddress = value; }

        [SerializeField]
        int m_RosPort = 10000;
        public int RosPort { get => m_RosPort; set => m_RosPort = value; }

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

        DateTime m_LastMessageReceived;
        DateTime m_LastMessageSent;
        public DateTime LastMessageReceived => m_LastMessageReceived;
        public DateTime LastMessageSent => m_LastMessageSent;

        // For the IP address field we show in the hud, we store the IP address and port in PlayerPrefs.
        // This is used to remember the last IP address the player typed into the HUD, in builds where ConnectOnStart is not checked
        public const string PlayerPrefsKey_ROS_IP = "ROS_IP";
        public const string PlayerPrefsKey_ROS_TCP_PORT = "ROS_TCP_PORT";
        public static string RosIPAddressPref => PlayerPrefs.GetString(PlayerPrefsKey_ROS_IP, "127.0.0.1");
        public static int RosPortPref => PlayerPrefs.GetInt(PlayerPrefsKey_ROS_TCP_PORT, 10000);

        public static void SetIPPref(string ipAddress)
        {
            PlayerPrefs.SetString(PlayerPrefsKey_ROS_IP, ipAddress);
        }

        public static void SetPortPref(int port)
        {
            PlayerPrefs.SetInt(PlayerPrefsKey_ROS_TCP_PORT, port);
        }

        class OutgoingMessageQueue
        {
            ConcurrentQueue<IOutgoingMessageSender> m_OutgoingMessageQueue;
            public readonly ManualResetEvent NewMessageReadyToSendEvent;

            public OutgoingMessageQueue()
            {
                m_OutgoingMessageQueue = new ConcurrentQueue<IOutgoingMessageSender>();
                NewMessageReadyToSendEvent = new ManualResetEvent(false);
            }

            public void Enqueue(IOutgoingMessageSender outgoingMessageSender)
            {
                m_OutgoingMessageQueue.Enqueue(outgoingMessageSender);
                NewMessageReadyToSendEvent.Set();
            }

            public bool TryDequeue(out IOutgoingMessageSender outgoingMessageSender)
            {
                return m_OutgoingMessageQueue.TryDequeue(out outgoingMessageSender);
            }
        }

        OutgoingMessageQueue m_OutgoingMessageQueue = new OutgoingMessageQueue();
        ConcurrentQueue<Tuple<string, byte[]>> m_IncomingMessages = new ConcurrentQueue<Tuple<string, byte[]>>();
        CancellationTokenSource m_ConnectionThreadCancellation;

        public bool HasConnection => m_ConnectionThreadCancellation != null;
        static bool m_HasConnectionError = false;
        static bool m_HasOutputConnectionError = false;
        public bool HasConnectionError => m_HasConnectionError;

        /*public Stream Connect()
        {
            if (!IPFormatIsCorrect(RosIPAddress))
                Debug.LogWarning("Invalid ROS IP address: " + RosIPAddress);

            TcpClient client = new TcpClient();
            client.Connect(m_RosIPAddress, m_RosPort);
            return client.GetStream();
        }*/

        public void Start()
        {
            HudPanel.RegisterHeader(DrawHeaderGUI);
        }

        ISerializationProvider m_SerializationProvider;
        Action<Stream> m_OnConnectionStartedCallback;
        Action m_OnConnectionLostCallback;

        public void Init(
            ISerializationProvider serializationProvider,
            Action<Stream> onConnectionStartedCallback,
            Action onConnectionLostCallback)
        {
            m_SerializationProvider = serializationProvider;
            m_OnConnectionStartedCallback = onConnectionStartedCallback;
            m_OnConnectionLostCallback = onConnectionLostCallback;
        }

        /*public void Connect(string ipAddress, int port)
        {
            RosIPAddress = ipAddress;
            RosPort = port;
            Connect();
        }*/

        public void Connect()
        {
            if (!IPFormatIsCorrect(RosIPAddress))
                Debug.LogWarning("Invalid ROS IP address: " + RosIPAddress);

            m_ConnectionThreadCancellation = new CancellationTokenSource();

            Task.Run(() => ConnectionThread(
                RosIPAddress,
                RosPort,
                m_SerializationProvider,
                m_NetworkTimeoutSeconds,
                m_KeepaliveTime,
                (int)(m_SleepTimeSeconds * 1000.0f),
                m_OnConnectionStartedCallback,
                m_OnConnectionLostCallback,
                SetHasConnectionError,
                m_OutgoingMessageQueue,
                m_IncomingMessages,
                m_ConnectionThreadCancellation.Token
            ));
        }

        void SetHasConnectionError(bool hasError)
        {
            m_HasConnectionError = hasError;
        }

        public void Disconnect()
        {
            m_ConnectionThreadCancellation?.Cancel();
            //The thread may be waiting on a ManualResetEvent, if so, this will wake it so it can exit immediately.
            m_OutgoingMessageQueue?.NewMessageReadyToSendEvent?.Set();
            m_ConnectionThreadCancellation = null;
        }

        static void ClearMessageQueue(OutgoingMessageQueue queue)
        {
            while (queue.TryDequeue(out IOutgoingMessageSender sendsOutgoingMessages))
            {
                sendsOutgoingMessages.ClearAllQueuedData();
            }
        }

        public bool TryRead(out string topic, out byte[] data)
        {
            Tuple<string, byte[]> tuple;
            if (m_IncomingMessages.TryDequeue(out tuple))
            {
                topic = tuple.Item1;
                data = tuple.Item2;
                m_LastMessageReceived = DateTime.Now;
                return true;
            }
            else
            {
                topic = null;
                data = null;
                return false;
            }
        }

        public void Send(string topic, string text)
        {
            m_OutgoingMessageQueue.Enqueue(new StringSender(topic, text));
        }

        public void Send(string topic, Message msg)
        {
            m_OutgoingMessageQueue.Enqueue(new SimpleMessageSender(topic, msg));
        }

        public void Send(IOutgoingMessageSender sender)
        {
            m_OutgoingMessageQueue.Enqueue(sender);
        }

        void DrawHeaderGUI()
        {
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
            };


            // ROS IP Setup
            GUILayout.BeginHorizontal(GUILayout.Width(300));
            DrawConnectionArrows(
                true,
                0,
                0,
                DateTime.Now - LastMessageReceived,
                DateTime.Now - LastMessageSent,
                HasConnection,
                HasConnection,
                HasConnectionError
            );

#if ROS2
            string protocolName = "ROS2";
#else
            string protocolName = "ROS";
#endif

            GUILayout.Space(30);
            GUILayout.Label($"{protocolName} IP: ", labelStyle, GUILayout.Width(100));

            if (!HasConnection)
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
                {
                    m_RosIPAddress = RosIPAddressPref;
                    m_RosPort = RosPortPref;
                    Connect();
                }
            }
            else
            {
                GUILayout.Label($"{RosIPAddress}:{RosPort}", contentStyle);

                if (HasConnectionError)
                {
                    if (GUI.Button(new Rect(250, 2, 50, 22), "Set IP"))
                        Disconnect();
                }

                GUILayout.EndHorizontal();
            }
        }

        static GUIStyle s_ConnectionArrowStyle;

        public static void DrawConnectionArrows(bool withBar, float x, float y, TimeSpan receivedTime, TimeSpan sentTime, bool isPublisher, bool isSubscriber, bool hasError)
        {
            if (s_ConnectionArrowStyle == null)
            {
                s_ConnectionArrowStyle = new GUIStyle
                {
                    alignment = TextAnchor.MiddleLeft,
                    normal = { textColor = Color.white },
                    fontSize = 22,
                    fontStyle = FontStyle.Bold,
                    fixedWidth = 250
                };
            }

            var baseColor = GUI.color;
            GUI.color = Color.white;
            if (withBar)
                GUI.Label(new Rect(x + 4, y + 5, 25, 15), "I", s_ConnectionArrowStyle);
            GUI.color = GetConnectionColor(receivedTime, isSubscriber, hasError);
            GUI.Label(new Rect(x + 8, y + 6, 25, 15), "\u2190", s_ConnectionArrowStyle);
            GUI.color = GetConnectionColor(sentTime, isPublisher, hasError);
            GUI.Label(new Rect(x + 8, y + 0, 25, 15), "\u2192", s_ConnectionArrowStyle);
            GUI.color = baseColor;
        }

        public static Color GetConnectionColor(TimeSpan elapsedTime, bool hasConnection, bool hasError)
        {
            var bright = new Color(1, 1, 0.5f);
            var mid = new Color(0, 1, 1);
            var dark = new Color(0, 0.5f, 1);
            const float brightDuration = 0.03f;
            const float fadeToDarkDuration = 1.0f;

            if (!hasConnection)
                return Color.gray;
            if (hasError)
                return Color.red;

            double timeSeconds = elapsedTime.TotalSeconds;
            if (timeSeconds <= brightDuration)
                return bright;
            return Color.Lerp(mid, dark, (float)timeSeconds / fadeToDarkDuration);
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

        static void SendKeepalive(NetworkStream stream)
        {
            // 8 zeroes = a ros message with topic "" and no message data.
            stream.Write(new byte[] { 0, 0, 0, 0, 0, 0, 0, 0 }, 0, 8);
        }

        static async Task ConnectionThread(
            string rosIPAddress,
            int rosPort,
            ISerializationProvider serializationProvider,
            float networkTimeoutSeconds,
            float keepaliveTime,
            int sleepMilliseconds,
            Action<NetworkStream> OnConnectionStartedCallback,
            Action DeregisterAll,
            Action<bool> SetHasConnectionError,
            OutgoingMessageQueue outgoingQueue,
            ConcurrentQueue<Tuple<string, byte[]>> incomingQueue,
            CancellationToken token)
        {
            //Debug.Log("ConnectionThread begins");
            int nextReaderIdx = 101;
            int nextReconnectionDelay = 1000;
            IMessageSerializer messageSerializer = serializationProvider.CreateSerializer();

            while (!token.IsCancellationRequested)
            {
                TcpClient client = null;
                CancellationTokenSource readerCancellation = null;

                try
                {
                    SetHasConnectionError(true); // until we actually see a reply back, assume there's a problem

                    client = new TcpClient();
                    client.Connect(rosIPAddress, rosPort);

                    NetworkStream networkStream = client.GetStream();
                    networkStream.ReadTimeout = (int)(networkTimeoutSeconds * 1000);

                    SendKeepalive(networkStream);
                    OnConnectionStartedCallback(networkStream);

                    readerCancellation = new CancellationTokenSource();
                    _ = Task.Run(() => ReaderThread(nextReaderIdx, networkStream, incomingQueue, sleepMilliseconds, SetHasConnectionError, readerCancellation.Token));
                    nextReaderIdx++;

                    if (m_HasOutputConnectionError)
                    {
                        Debug.Log($"ROS Connection to {rosIPAddress}:{rosPort} succeeded!");
                        m_HasOutputConnectionError = false;
                    }

                    // connected, now just watch our queue for outgoing messages to send (or else send a keepalive message occasionally)
                    DateTime waitingSince = DateTime.Now;
                    while (true)
                    {
                        bool messageReadyEventWasSet = outgoingQueue.NewMessageReadyToSendEvent.WaitOne(sleepMilliseconds);
                        token.ThrowIfCancellationRequested();

                        if (messageReadyEventWasSet)
                        {
                            outgoingQueue.NewMessageReadyToSendEvent.Reset();
                        }
                        else
                        {
                            if ((DateTime.Now - waitingSince).TotalSeconds > keepaliveTime)
                            {
                                SendKeepalive(networkStream);
                                waitingSince = DateTime.Now;
                            }
                        }

                        while (outgoingQueue.TryDequeue(out IOutgoingMessageSender sendsOutgoingMessages))
                        {
                            IOutgoingMessageSender.SendToState sendToState = sendsOutgoingMessages.SendInternal(messageSerializer, networkStream);
                            switch (sendToState)
                            {
                                case IOutgoingMessageSender.SendToState.Normal:
                                    //This is normal operation.
                                    break;
                                case IOutgoingMessageSender.SendToState.QueueFullWarning:
                                    //Unable to send messages to ROS as fast as we're generating them.
                                    //This could be caused by a TCP connection that is too slow.
                                    Debug.LogWarning($"Queue full! Messages are getting dropped! " +
                                                     "Try check your connection speed is fast enough to handle the traffic.");
                                    break;
                                case IOutgoingMessageSender.SendToState.NoMessageToSendError:
                                    //This indicates
                                    Debug.LogError(
                                        "Logic Error! An 'IOutgoingMessageSender' was queued but did not have any messages to send.");
                                    break;
                            }

                            token.ThrowIfCancellationRequested();
                            waitingSince = DateTime.Now;
                        }
                    }
                }
                catch (OperationCanceledException)
                {
                }
                catch (Exception e)
                {
                    SetHasConnectionError(true);
                    if (!m_HasOutputConnectionError)
                    {
                        Debug.LogError($"ROS Connection to {rosIPAddress}:{rosPort} failed - " + e);
                        m_HasOutputConnectionError = true;
                    }
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

        static async Task ReaderThread(
            int readerIdx,
            Stream networkStream,
            ConcurrentQueue<Tuple<string, byte[]>> queue,
            int sleepMilliseconds,
            Action<bool> SetHasConnectionError,
            CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                try
                {
                    Tuple<string, byte[]> content = await ReadMessageContents(networkStream, sleepMilliseconds, token);
                    SetHasConnectionError(false);

                    if (content.Item1 != "") // ignore keepalive messages
                        queue.Enqueue(content);
                }
                catch (OperationCanceledException)
                {
                }
                catch (Exception e)
                {
                    SetHasConnectionError(true);
                    Debug.Log("Reader " + readerIdx + " exception! " + e);
                }
            }
        }

        static async Task ReadToByteArray(System.IO.Stream networkStream, byte[] array, int length, int sleepMilliseconds, CancellationToken token)
        {
            int read = 0;
            while (read < length && networkStream.CanRead)
            {
                token.ThrowIfCancellationRequested();
                read += await networkStream.ReadAsync(array, read, length - read, token);
            }

            if (read < length)
                throw new SocketException(); // the connection has closed
        }

        static byte[] s_FourBytes = new byte[4];
        static byte[] s_TopicScratchSpace = new byte[64];

        static async Task<Tuple<string, byte[]>> ReadMessageContents(System.IO.Stream networkStream, int sleepMilliseconds, CancellationToken token)
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
    }
}
