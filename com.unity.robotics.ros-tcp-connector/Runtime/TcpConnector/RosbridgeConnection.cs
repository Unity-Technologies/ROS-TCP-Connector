using NativeWebSocket;
using Newtonsoft.Json.Linq;
using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class RosbridgeConnection : MonoBehaviour, IConnection
    {
#if ROS2
        [SerializeField]
        bool m_IsRos2 = true;
#else
        [SerializeField]
        bool m_IsRos2 = false;
#endif
        [SerializeField]
        string m_Address = "ws://localhost:9090";
        public string Address { get => m_Address; set => m_Address = value; }

        WebSocket m_Websocket;
        Queue<string> m_DeferredMessages = new Queue<string>();

        Dictionary<string, RosbridgeSubscriber> m_Subscribers = new Dictionary<string, RosbridgeSubscriber>();
        bool m_IsConnecting = false;
        bool m_IsConnected = false;
        bool m_HasConnectionError = false;
        JsonSerializer m_JsonSerializer;
        JsonDeserializer m_JsonDeserializer;

        float m_LastMessageSentRealtime;
        float m_LastMessageReceivedRealtime;

        [SerializeField]
        bool m_ShowInHud = true;
        public bool ShowInHud { get => m_ShowInHud; set => m_ShowInHud = value; }

        [SerializeField]
        bool m_ConnectOnStart = true;
        public bool ConnectOnStart { get => m_ConnectOnStart; set => m_ConnectOnStart = value; }

        void Awake()
        {
            m_JsonSerializer = new JsonSerializer(m_IsRos2);
            m_JsonDeserializer = new JsonDeserializer(m_IsRos2);
        }

        void Start()
        {
            if (ShowInHud)
                ConnectionHud.RegisterHeader(DrawHeaderGUI);

            if (ConnectOnStart)
                Connect();
        }

        public void Connect(string address)
        {
            if (!m_IsConnecting)
            {
                m_Address = address;
                Connect();
            }
        }

        public async void Connect()
        {
            m_IsConnecting = true;
            m_Websocket = new WebSocket(m_Address);
            m_Websocket.OnOpen += OnSocketConnected;
            m_Websocket.OnError += OnSocketError;
            /*            m_Websocket.OnError += (e) =>
                        {
                            Debug.Log("Error! " + e);
                        };

                        m_Websocket.OnClose += (e) =>
                        {
                            Debug.Log("Connection closed!");
                        };*/

            m_Websocket.OnMessage += OnMessageReceived;

            await m_Websocket.Connect();
        }

        void OnSocketConnected()
        {
            m_HasConnectionError = false;
            m_IsConnected = true;
            while (m_DeferredMessages.Count > 0)
            {
                m_Websocket.SendText(m_DeferredMessages.Dequeue());
            }
            m_DeferredMessages.Clear();
        }

        void OnSocketError(string error)
        {
            m_HasConnectionError = true;
            Debug.LogError("Rosbridge Connection error: " + error);
        }

        void OnMessageReceived(byte[] bytes)
        {
            m_HasConnectionError = false;
            JObject message = JObject.Parse(Encoding.UTF8.GetString(bytes));
            JToken opToken = (string)message["op"];
            if (opToken != null)
            {
                switch ((string)opToken)
                {
                    case "publish":
                        {
                            // we received a published message from a topic we subscribed to (hopefully)
                            RosbridgeSubscriber subscriber;
                            if (m_Subscribers.TryGetValue((string)message["topic"], out subscriber))
                            {
                                subscriber.OnMessageReceived((JObject)message["msg"]);
                            }
                        }
                        break;
                }
                //Debug.Log("Received <" + (string)opToken + ">");
            }

        }

        public void Disconnect()
        {
            m_Websocket.Close();
            m_IsConnecting = false;
        }

        public IPublisher RegisterPublisher<T>(string topic) where T : Message
        {
            return RegisterPublisher(topic, MessageRegistry.GetMessageTypeString<T>());
        }

        public IPublisher RegisterPublisher(string topic, string messageType)
        {
            string json = $"{{\"op\":\"advertise\",\"topic\":\"{topic}\",\"type\":\"{messageType}\"}}";
            Send(json);
            return new RosbridgePublisher(topic, messageType, this);
        }

        public void Subscribe<T>(string topic, Action<T> callback) where T : Message
        {
            Subscribe(topic, MessageRegistry.GetMessageTypeString<T>(), (Message msg) => callback((T)msg));
        }

        public void Subscribe(string topic, string messageType, Action<Message> callback)
        {
            RosbridgeSubscriber subscriber;
            if (!m_Subscribers.TryGetValue(topic, out subscriber))
            {
                subscriber = new RosbridgeSubscriber(m_JsonDeserializer, messageType);
                m_Subscribers.Add(topic, subscriber);
                string json = $"{{\"op\":\"subscribe\",\"topic\":\"{topic}\",\"type\":\"{messageType}\"}}";
                Send(json);
            }
            subscriber.AddCallback(callback);
        }

        void Send(string json)
        {
            if (m_IsConnected)
                m_Websocket.SendText(json);
            else
                m_DeferredMessages.Enqueue(json);
        }

        void Update()
        {
            m_Websocket.DispatchMessageQueue();
        }

        public class RosbridgePublisher : IPublisher
        {
            string m_Topic;
            public string Topic => m_Topic;

            string m_MessageType;
            public string MessageType => m_MessageType;

            RosbridgeConnection m_Connection;
            public RosbridgePublisher(string topic, string type, RosbridgeConnection connection)
            {
                m_Topic = topic;
                m_MessageType = type;
                m_Connection = connection;
            }

            public void Publish(Message msg)
            {
                string msgjson = m_Connection.m_JsonSerializer.ToJsonString(msg);
                string json = $"{{\"op\":\"publish\",\"topic\":\"{m_Topic}\",\"msg\":{msgjson}}}";
                m_Connection.Send(json);
            }
        }

        class RosbridgeSubscriber
        {
            Func<JObject, Message> m_Deserialize;
            public readonly string m_MessageTypeName;
            List<Action<Message>> m_Callbacks = new List<Action<Message>>();

            public RosbridgeSubscriber(JsonDeserializer messageDeserializer, string messageTypeName)
            {
                this.m_MessageTypeName = messageTypeName;
                var invoker = MessageRegistry.GetGenericInvoker(messageTypeName);
                m_Deserialize = json => invoker(messageDeserializer, json);
            }

            public void AddCallback(Action<Message> callback)
            {
                m_Callbacks.Add(callback);
            }

            public void OnMessageReceived(JObject message)
            {
                if (m_Callbacks.Count > 0)
                {
                    Message msg = m_Deserialize(message);
                    foreach (var callback in m_Callbacks)
                        callback(msg);
                }
            }
        }

        static string PlayerPrefsKey_Rosbridge_Address = "Rosbridge_Address";

        public static void SetAddressPref(string address)
        {
            PlayerPrefs.SetString(PlayerPrefsKey_Rosbridge_Address, address);
        }

        public static string GetAddressPref() => PlayerPrefs.GetString(PlayerPrefsKey_Rosbridge_Address);

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
            ConnectionHud.DrawConnectionArrows(
                true,
                0,
                0,
                Time.realtimeSinceStartup - m_LastMessageReceivedRealtime,
                Time.realtimeSinceStartup - m_LastMessageSentRealtime,
                m_IsConnected,
                m_IsConnected,
                m_HasConnectionError
            );

            string protocolName = m_IsRos2 ? "ROS2" : "ROS";

            GUILayout.Space(30);
            GUILayout.Label($"{protocolName} bridge IP: ", labelStyle, GUILayout.Width(100));

            if (!m_IsConnecting)
            {
                // if you've never run a build on this machine before, initialize the playerpref settings to the ones from the RosConnection
                if (!PlayerPrefs.HasKey(PlayerPrefsKey_Rosbridge_Address))
                    SetAddressPref(m_Address);

                // NB, here the user is editing the PlayerPrefs values, not the ones in the RosConnection.
                // (So that the hud remembers what IP you used last time you ran this build.)
                // The RosConnection receives the edited values when you click Connect.
                SetAddressPref(GUILayout.TextField(GetAddressPref()));

                GUILayout.EndHorizontal();
                GUILayout.Label("(Not connected)");
                if (GUILayout.Button("Connect"))
                    Connect(GetAddressPref());
            }
            else
            {
                GUILayout.Label($"{m_Address}", contentStyle);

                if (m_HasConnectionError)
                {
                    if (GUI.Button(new Rect(250, 2, 50, 22), "Set IP"))
                        Disconnect();
                }

                GUILayout.EndHorizontal();
            }
        }
    }
}
