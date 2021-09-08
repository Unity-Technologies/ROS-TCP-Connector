using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class RosTopicState
    {
        string m_Topic;
        public string Topic => m_Topic;

        MessageSubtopic m_Subtopic;
        public MessageSubtopic Subtopic => m_Subtopic;

        string m_RosMessageName;
        public string RosMessageName => m_RosMessageName;

        RosPublisher m_Publisher;
        public RosPublisher Publisher => m_Publisher;
        public bool IsPublisher => m_Publisher != null;

        bool m_IsRosService;
        public bool IsRosService => m_IsRosService;

        ROSConnection m_Connection;
        public ROSConnection Connection => m_Connection;

        ROSConnection.InternalAPI m_ConnectionInternal;
        Func<MessageDeserializer, Message> m_Deserializer;
        Func<Message, Message> m_ServiceImplementation;
        RosTopicState m_ServiceResponseTopic;
        public RosTopicState ServiceResponseTopic => m_ServiceResponseTopic;

        public bool IsUnityService => m_ServiceImplementation != null;

        List<Action<Message>> m_SubscriberCallbacks = new List<Action<Message>>();
        public bool HasSubscriberCallback => m_SubscriberCallbacks.Count > 0;
        bool m_SentSubscriberRegistration;
        public bool SentSubscriberRegistration => m_SentSubscriberRegistration;

        float m_LastMessageReceivedRealtime;
        float m_LastMessageSentRealtime;
        public float LastMessageReceivedRealtime => m_LastMessageReceivedRealtime;
        public float LastMessageSentRealtime => m_LastMessageSentRealtime;

        internal RosTopicState(string topic, string rosMessageName, ROSConnection connection, ROSConnection.InternalAPI connectionInternal, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            m_Topic = topic;
            m_Subtopic = subtopic;
            m_RosMessageName = rosMessageName;
            m_Connection = connection;
            m_ConnectionInternal = connectionInternal;
        }

        public void ChangeRosMessageName(string rosMessageName)
        {
            if (m_RosMessageName != null)
                Debug.LogWarning($"Inconsistent declaration of topic '{Topic}': was '{m_RosMessageName}', switching to '{rosMessageName}'.");
            m_RosMessageName = rosMessageName;
            // force deserializer to be refreshed
            m_Deserializer = null;
        }

        public void OnMessageReceived(byte[] data)
        {
            m_LastMessageReceivedRealtime = Time.realtimeSinceStartup;
            if (m_IsRosService && m_ServiceResponseTopic != null)
            {
                //  For a service, incoming messages are a different type from outgoing messages.
                //  We process them using a separate RosTopicState.
                m_ServiceResponseTopic.OnMessageReceived(data);
                return;
            }

            // don't bother deserializing this message if nobody cares
            if (m_SubscriberCallbacks.Count == 0)
            {
                return;
            }

            Message message = Deserialize(data);

            m_SubscriberCallbacks.ForEach(item => item(message));
        }

        public void OnMessageSent(Message message)
        {
            m_LastMessageSentRealtime = Time.realtimeSinceStartup;
            if (m_RosMessageName == null)
            {
                ChangeRosMessageName(message.RosMessageName);
            }

            m_SubscriberCallbacks.ForEach(item => item(message));
        }

        public Message HandleUnityServiceRequest(byte[] data)
        {
            if (m_ServiceImplementation == null)
            {
                Debug.LogError($"Unity service '{m_Topic}' has not been implemented!");
                return null;
            }

            // deserialize the request message
            Message requestMessage = Deserialize(data);

            // run the actual service
            Message response = m_ServiceImplementation(requestMessage);
            m_ServiceResponseTopic.OnMessageSent(response);
            return response;
        }

        Message Deserialize(byte[] data)
        {
            if (m_Deserializer == null)
                m_Deserializer = MessageRegistry.GetDeserializeFunction(m_RosMessageName, m_Subtopic);

            m_ConnectionInternal.Deserializer.InitWithBuffer(data);
            return m_Deserializer(m_ConnectionInternal.Deserializer);
        }

        public void AddSubscriber(Action<Message> callback)
        {
            m_SubscriberCallbacks.Add(callback);

            RegisterSubscriber();
        }

        void RegisterSubscriber(NetworkStream stream = null)
        {
            if (m_Connection.HasConnectionThread && !m_SentSubscriberRegistration)
            {
                m_ConnectionInternal.SendSubscriberRegistration(m_Topic, m_RosMessageName, stream);
                m_SentSubscriberRegistration = true;
            }
        }

        public void UnsubscribeAll()
        {
            m_SubscriberCallbacks.Clear();
            m_ConnectionInternal.SendSubscriberUnregistration(m_Topic);
            m_SentSubscriberRegistration = false;
        }

        public void ImplementService(Func<Message, Message> implementation)
        {
            m_ServiceImplementation = implementation;
            m_ConnectionInternal.SendUnityServiceRegistration(m_Topic, m_RosMessageName);
            m_ServiceResponseTopic = new RosTopicState(m_Topic, null, m_Connection, m_ConnectionInternal, MessageSubtopic.Response);
        }

        public RosPublisher CreatePublisher(int queueSize, bool latch)
        {
            m_Publisher = new RosPublisher(Topic, m_RosMessageName, queueSize, latch);
            return m_Publisher;
        }

        public void OnDeregister()
        {
            if (m_Publisher != null)
                m_Publisher.OnConnectionLost();
        }

        public void RegisterRosService(string responseMessageName)
        {
            m_IsRosService = true;
            m_ConnectionInternal.SendRosServiceRegistration(m_Topic, m_RosMessageName);
            m_ServiceResponseTopic = new RosTopicState(m_Topic, responseMessageName, m_Connection, m_ConnectionInternal, MessageSubtopic.Response);
        }

        public void RegisterAll(NetworkStream stream)
        {
            if (m_SubscriberCallbacks.Count > 0)
            {
                m_ConnectionInternal.SendSubscriberRegistration(m_Topic, m_RosMessageName, stream);
                m_SentSubscriberRegistration = true;
            }

            if (m_ServiceImplementation != null)
            {
                m_ConnectionInternal.SendUnityServiceRegistration(m_Topic, m_RosMessageName, stream);
            }

            if (m_Publisher != null)
            {
                m_Publisher.OnConnectionEstablished(stream);
            }

            if (m_IsRosService)
            {
                m_ConnectionInternal.SendRosServiceRegistration(m_Topic, m_RosMessageName, stream);
            }
        }
    }
}
