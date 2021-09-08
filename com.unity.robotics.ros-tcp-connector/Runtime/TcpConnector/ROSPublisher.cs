using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class RosPublisher : OutgoingMessageSender
    {
        public string RosMessageName { get; private set; }

        public string TopicName { get; private set; }

        public int QueueSize { get; private set; }

        public bool Latch { get; private set; }

        public bool PublisherRegistered { get; private set; }

        //Messages waiting to be sent queue.
        LinkedList<Message> m_OutgoingMessages = new LinkedList<Message>();

        //Keeps track of how many outgoing messages were removed due to queue overflow.
        //If a message is published but the queue is full, the counter is incremented, the
        //first message in the queue is recycled and the message to publish is put on the end of the queue.
        //If this is non 0, a SendTo call will decrement the counter instead of sending data.
        int m_QueueOverflowUnsentCounter = 0;

        //Latching - This message will be set if latching is enabled, and on reconnection, it will be sent again.
        Message m_LastMessageSent = null;

        //Optional, used if you want to pool messages and reuse them when they are no longer in use.
        Queue<Message> m_InactiveMessagePool = new Queue<Message>();

        public RosPublisher(string topicName, string rosMessageName, int queueSize, bool latch)
        {
            if (queueSize < 1)
            {
                throw new Exception("Queue size must be greater than or equal to 1.");
            }

            RosMessageName = rosMessageName;
            TopicName = topicName;
            QueueSize = queueSize;
            Latch = latch;
            PublisherRegistered = false;
        }

        public bool EquivalentTo(string topicName, string messageName, int? queueSize, bool? latch)
        {
            return TopicName == topicName && RosMessageName == messageName &&
                   (!queueSize.HasValue || QueueSize == queueSize) &&
                   (!latch.HasValue || Latch == latch);
        }

        internal void PublishInternal(Message message)
        {
            lock (m_OutgoingMessages)
            {
                if (m_OutgoingMessages.Count >= QueueSize)
                {
                    //Remove outgoing messages that don't fit in the queue.
                    //Recycle the message if applicable
                    RecycleMessageIfApplicable(m_OutgoingMessages.First.Value);
                    //Update the overflow counter.
                    m_QueueOverflowUnsentCounter++;
                    m_OutgoingMessages.RemoveFirst();
                }

                //Add a new valid message to the end.
                m_OutgoingMessages.AddLast(message);
            }
        }

        public void OnConnectionEstablished(MessageSerializer m_MessageSerializer, Stream stream)
        {
            //Register the publisher with the ROS Endpoint.
            RegisterPublisherIfApplicable(m_MessageSerializer, stream);
        }

        public void OnConnectionLost()
        {
            PublisherRegistered = false;
        }

        SendToState RemoveMessageToSend(out Message messageToSend)
        {
            SendToState result = SendToState.NoMessageToSendError;
            messageToSend = null;
            lock (m_OutgoingMessages)
            {
                if (m_QueueOverflowUnsentCounter > 0)
                {
                    //This means that we can't send message to ROS as fast as we're generating them.
                    //This could potentially be bad as it means that we are dropping messages!
                    m_QueueOverflowUnsentCounter--;
                    messageToSend = null;
                    result = SendToState.QueueFullWarning;
                }
                else if (m_OutgoingMessages.Count > 0)
                {
                    //Retrieve the next message and populate messageToSend.
                    messageToSend = m_OutgoingMessages.First.Value;
                    m_OutgoingMessages.RemoveFirst();
                    result = SendToState.Normal;
                }
            }

            return result;
        }

        public bool PeekNextMessageToSend(out Message messageToSend)
        {
            bool result = false;
            messageToSend = null;
            lock (m_OutgoingMessages)
            {
                if (m_OutgoingMessages.Count > 0)
                {
                    messageToSend = m_OutgoingMessages.First.Value;
                    result = true;
                }
            }


            return result;
        }

        void SendMessageWithStream(MessageSerializer messageSerializer, Stream stream, Message message)
        {
            RegisterPublisherIfApplicable(messageSerializer, stream);

            //Clear the serializer
            messageSerializer.Clear();
            //Prepare the data to send.
            messageSerializer.Write(TopicName);
            messageSerializer.SerializeMessageWithLength(message);
            //Send via the stream.
            messageSerializer.SendTo(stream);
        }

        void RegisterPublisherIfApplicable(MessageSerializer messageSerializer, Stream stream)
        {
            if (PublisherRegistered)
            {
                return;
            }

            //Register the publisher before sending anything.
            SysCommandPublisherRegistration publisherRegistration =
                new SysCommandPublisherRegistration(this);
            publisherRegistration.SendTo(stream, messageSerializer);
            PublisherRegistered = true;

            if (Latch && m_LastMessageSent != null)
            {
                //This topic is latching, so to mimic that functionality,
                //here the last sent message is sent again with the new connection.
                SendMessageWithStream(messageSerializer, stream, m_LastMessageSent);
            }
        }

        internal override SendToState SendInternal(MessageSerializer m_MessageSerializer, Stream stream)
        {
            RegisterPublisherIfApplicable(m_MessageSerializer, stream);
            SendToState sendToState = RemoveMessageToSend(out Message toSend);
            if (sendToState == SendToState.Normal)
            {
                SendMessageWithStream(m_MessageSerializer, stream, toSend);

                //Recycle the message (if applicable).
                if (Latch)
                {
                    if (m_LastMessageSent != null && m_LastMessageSent != toSend)
                    {
                        RecycleMessageIfApplicable(m_LastMessageSent);
                    }

                    m_LastMessageSent = toSend;
                }
                else
                {
                    RecycleMessageIfApplicable(toSend);
                }
            }

            return sendToState;
        }

        public override void ClearAllQueuedData()
        {
            List<Message> toRecycle;
            lock (m_OutgoingMessages)
            {
                toRecycle = new List<Message>(m_OutgoingMessages);
                m_OutgoingMessages.Clear();
                m_QueueOverflowUnsentCounter = 0;
            }

            foreach (Message messageToRecycle in toRecycle)
            {
                RecycleMessageIfApplicable(messageToRecycle);
            }
        }

        void RecycleMessageIfApplicable(Message toRecycle)
        {
            if (!MessagePoolEnabled)
            {
                return;
            }

            //Add the message back to the pool.
            AddMessageToPool(toRecycle);
        }

        #region Message Pooling

        //Whether you want to pool messages for reuse (Used to reduce GC calls).
        volatile bool m_MessagePoolEnabled;

        public bool MessagePoolEnabled => m_MessagePoolEnabled;

        public void SetMessagePoolEnabled(bool enabled)
        {
            if (m_MessagePoolEnabled == enabled)
                return;

            m_MessagePoolEnabled = enabled;
            if (!m_MessagePoolEnabled)
            {
                lock (m_InactiveMessagePool)
                {
                    m_InactiveMessagePool.Clear();
                }
            }
        }

        public void AddMessageToPool(Message messageToRecycle)
        {
            Debug.Assert(MessagePoolEnabled,
                "Adding a message to a message pool that is not enabled, please set MessagePoolEnabled to true.");

            lock (m_InactiveMessagePool)
            {
                if (MessagePoolEnabled && m_InactiveMessagePool.Count < (QueueSize + 5))
                {
                    //Make sure we're only pooling a reasonable amount.
                    //We shouldn't need any more than the queue size plus a little.
                    m_InactiveMessagePool.Enqueue(messageToRecycle);
                }
            }
        }

        /**
         * @return a message of type T full of garbage data, be sure to update it accordingly.
         */
        public Message GetMessageFromPool()
        {
            Message result = null;
            SetMessagePoolEnabled(true);
            lock (m_InactiveMessagePool)
            {
                if (m_InactiveMessagePool.Count > 0)
                {
                    result = m_InactiveMessagePool.Dequeue();
                }
            }

            if (result == null)
            {
                result = Activator.CreateInstance<Message>();
            }

            return result;
        }

        #endregion
    }
}
