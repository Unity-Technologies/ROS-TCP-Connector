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
        bool m_IsConnected = false;
        JsonSerializer m_JsonSerializer;
        JsonDeserializer m_JsonDeserializer;

        void Awake()
        {
            m_JsonSerializer = new JsonSerializer(m_IsRos2);
            m_JsonDeserializer = new JsonDeserializer(m_IsRos2);
        }

        public async void Connect()
        {
            m_Websocket = new WebSocket(m_Address);
            m_Websocket.OnOpen += OnSocketOpen;
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

        void OnSocketOpen()
        {
            while (m_DeferredMessages.Count > 0)
            {
                m_Websocket.SendText(m_DeferredMessages.Dequeue());
            }
            m_DeferredMessages.Clear();
            m_IsConnected = true;
        }

        void OnMessageReceived(byte[] bytes)
        {
            JObject message = JObject.Parse(Encoding.UTF8.GetString(bytes));
            JToken opToken = (string)message["op"];
            if (opToken != null)
            {
                Debug.Log("Message received, " + opToken);
                switch ((string)opToken)
                {
                    case "publish":
                        {
                            Debug.Log("Publish received");
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
                string json = $"{{\"op\":\"subscribe\",\"topic\":\"{topic}\"}}";//,\"type\":\"{messageType}\"}}";
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
    }
}
