using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class RosTopicState
    {
        string m_Topic;
        public string Topic => m_Topic;

        string m_RosMessageName;
        public string RosMessageName => m_RosMessageName;

        bool m_CanPublish;
        public bool IsPublisher => m_CanPublish;

        bool m_IsRosService;
        public bool IsRosService => m_IsRosService;

        ROSConnection m_Connection;
        ROSConnection.InternalAPI m_ConnectionInternal;
        Func<MessageDeserializer, Message> m_Deserializer;
        Func<Message, Message> m_ServiceImplementation;
        public bool IsUnityService => m_ServiceImplementation != null;

        List<Action<Message>> m_Subscribers = new List<Action<Message>>();
        public bool HasSubscriber => m_Subscribers.Count > 0;

        IVisualFactory m_Visualizer;
        public IVisualFactory Visualizer => m_Visualizer;
        IVisual m_Visual;
        public IVisual Visual => m_Visual;
        HudWindow m_VisualWindow;
        bool m_IsVisualizingUI;
        public bool IsVisualizingUI => m_IsVisualizingUI;
        bool m_IsVisualizingDrawing;
        public bool IsVisualizingDrawing => m_IsVisualizingDrawing;
        float m_LastVisualFrameTime;

        internal RosTopicState(string topic, string rosMessageName, ROSConnection connection, ROSConnection.InternalAPI connectionInternal)
        {
            m_Topic = topic;
            m_RosMessageName = rosMessageName;
            m_Connection = connection;
            m_ConnectionInternal = connectionInternal;
        }

        [Serializable]
        public class SaveState
        {
            public Rect Rect;
            public string Topic;
            public string RosMessageName;
            public bool ShowWindow;
            public bool ShowDrawing;
        }

        internal RosTopicState(SaveState save, ROSConnection connection, ROSConnection.InternalAPI connectionInternal)
        {
            m_Topic = save.Topic;
            m_RosMessageName = save.RosMessageName;
            m_VisualWindow = new HudWindow(m_Topic, save.Rect);
            m_IsVisualizingUI = save.ShowWindow;
            m_IsVisualizingDrawing = save.ShowDrawing;
            m_Connection = connection;
            m_ConnectionInternal = connectionInternal;
        }

        public SaveState CreateSaveState()
        {
            if (!m_IsVisualizingUI && !m_IsVisualizingDrawing)
                return null;

            return new SaveState
            {
                Rect = m_VisualWindow.WindowRect,
                Topic = Topic,
                RosMessageName = RosMessageName,
                ShowWindow = m_IsVisualizingUI,
                ShowDrawing = m_IsVisualizingDrawing,
            };
        }

        public void OnMessageReceived(byte[] data)
        {
            bool shouldVisualize = Time.time > m_LastVisualFrameTime && (m_IsVisualizingUI || m_IsVisualizingDrawing);

            // don't bother deserializing this message if nobody cares
            if (m_Subscribers.Count == 0 && !shouldVisualize)
                return;

            Message message = Deserialize(data);

            m_Subscribers.ForEach(item => item(message));

            if (shouldVisualize)
            {
                UpdateVisual(message);
            }
        }

        public void OnMessageSent(Message message)
        {
            if (Time.time > m_LastVisualFrameTime && (m_IsVisualizingUI || m_IsVisualizingDrawing))
            {
                UpdateVisual(message);
            }
        }

        void UpdateVisual(Message message)
        {
            MessageMetadata meta = new MessageMetadata(m_Topic, Time.time, DateTime.Now);
            IVisual newVisual = m_Visualizer.CreateVisual(message, meta);
            newVisual.Recycle(m_Visual);
            m_Visual = newVisual;
            m_VisualWindow.SetGuiDrawer(m_Visual.OnGUI);
            m_LastVisualFrameTime = Time.time;
        }

        public Message ProcessServiceRequest(byte[] data)
        {
            if (m_ServiceImplementation == null)
            {
                Debug.LogError($"Unity service '{m_Topic}' has not been implemented!");
                return null;
            }

            // deserialize the request message
            Message requestMessage = Deserialize(data);

            // run the actual service
            return m_ServiceImplementation(requestMessage);
        }

        Message Deserialize(byte[] data)
        {
            if (m_Deserializer == null)
                m_Deserializer = MessageRegistry.GetDeserializeFunction(m_RosMessageName);

            m_ConnectionInternal.Deserializer.InitWithBuffer(data);
            return m_Deserializer(m_ConnectionInternal.Deserializer);
        }

        public void AddSubscriber(Action<Message> callback)
        {
            m_Subscribers.Add(callback);

            if (m_Connection.HasConnectionThread && m_Subscribers.Count == 1)
                m_ConnectionInternal.SendSubscriberRegistration(m_Topic, m_RosMessageName);
        }

        public void UnsubscribeAll()
        {
            m_Subscribers.Clear();
            m_ConnectionInternal.SendSubscriberUnregistration(m_Topic);
        }

        public void ImplementService(Func<Message, Message> implementation)
        {
            m_ServiceImplementation = implementation;
            m_ConnectionInternal.SendUnityServiceRegistration(m_Topic, m_RosMessageName);
        }

        public void RegisterPublisher()
        {
            m_CanPublish = true;
            m_ConnectionInternal.SendPublisherRegistration(m_Topic, m_RosMessageName);
        }

        public void RegisterRosService()
        {
            m_IsRosService = true;
            m_ConnectionInternal.SendRosServiceRegistration(m_Topic, m_RosMessageName);
        }

        public void RegisterAll(NetworkStream stream)
        {
            if (m_Subscribers.Count > 0)
            {
                m_ConnectionInternal.SendSubscriberRegistration(m_Topic, m_RosMessageName, stream);
            }

            if (m_ServiceImplementation != null)
            {
                m_ConnectionInternal.SendUnityServiceRegistration(m_Topic, m_RosMessageName, stream);
            }

            if (m_CanPublish)
            {
                m_ConnectionInternal.SendPublisherRegistration(m_Topic, m_RosMessageName, stream);
            }

            if (m_IsRosService)
            {
                m_ConnectionInternal.SendRosServiceRegistration(m_Topic, m_RosMessageName, stream);
            }
        }
    }
}
