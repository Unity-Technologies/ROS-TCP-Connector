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
        [SerializeField]
        bool m_IsRos2;
        [SerializeField]
        string m_Address = "ws://localhost:9090";
        public string Address { get => m_Address; set => m_Address = value; }

        WebSocket m_Websocket;
        Queue<string> m_DeferredMessages = new Queue<string>();
        Dictionary<string, List<Action<Message>>> m_SubscriberCallbacks = new Dictionary<string, List<Action<Message>>>();
        bool m_IsConnected = false;
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
            m_IsConnected = true;
        }

        void OnMessageReceived(byte[] bytes)
        {
            JObject message = JObject.Parse(Encoding.UTF8.GetString(bytes));
            JToken opToken = (string)message["op"];
            if (opToken != null)
            {
                switch ((string)opToken)
                {
                    case "publish":
                        {
                            // we subscribed and received a published message
                            List<Action<Message>> callbacks;
                            if (m_SubscriberCallbacks.TryGetValue((string)message["topic"], out callbacks))
                            {
                                string typename = "std_msgs/String";// (string)message["type"];
                                Func<JObject, Message> deserializer = MessageRegistry.GetJsonDeserializeFunction(typename);
                                Message msg = deserializer((JObject)message["msg"]);
                                foreach (var callback in callbacks)
                                    callback(msg);
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
            return RegisterPublisher(topic, MessageRegistry.GetRosMessageName<T>());
        }

        public IPublisher RegisterPublisher(string topic, string messageType)
        {
            string json = $"{{\"op\":\"advertise\",\"topic\":\"{topic}\",\"type\":\"{messageType}\"}}";
            Send(json);
            return new RosbridgePublisher(topic, messageType, this);
        }

        public void Subscribe<T>(string topic, Action<T> callback) where T : Message
        {
            Subscribe(topic, MessageRegistry.GetRosMessageName<T>(), (Message msg) => callback((T)msg));
        }

        public void Subscribe(string topic, string messageType, Action<Message> callback)
        {
            List<Action<Message>> callbacks;
            if (!m_SubscriberCallbacks.TryGetValue(topic, out callbacks))
            {
                callbacks = new List<Action<Message>>();
                m_SubscriberCallbacks.Add(topic, callbacks);
                Debug.Log("Sending subscribe");
                string json = $"{{\"op\":\"subscribe\",\"topic\":\"{topic}\"}}";//,\"type\":\"{messageType}\"}}";
                Send(json);
            }
            callbacks.Add(callback);
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

        class RosbridgePublisher : IPublisher
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
                string json = $"{{\"op\":\"publish\",\"topic\":\"{m_Topic}\",\"msg\":{JsonUtility.ToJson(msg)}}}";
                m_Connection.Send(json);
            }
        }
    }
}
