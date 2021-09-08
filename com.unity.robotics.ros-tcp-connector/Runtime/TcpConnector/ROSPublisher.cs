using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class ROSPublisher : OutgoingMessageSender
    {
        public string RosMessageName { get; }

        public bool PublisherRegistered { get; set; }

        //Messages waiting to be sent queue.
        private LinkedList<Message> outgoingMessages = new LinkedList<Message>();

        //Keeps track of how many outgoing messages were removed due to queue overflow.
        //If a message is published but the queue is full, the counter is incremented, the
        //first message in the queue is recycled and the message to publish is put on the end of the queue.
        //If this is non 0, a SendTo call will decrement the counter instead of sending data.
        private int queueOverflowUnsentCounter = 0;

        //Latching - This message will be set if latching is enabled, and on reconnection, it will be sent again.
        private Message lastMessageSent = null;

        //Whether you want to pool messages for reuse (Used to reduce GC calls).
        private volatile bool _messagePoolEnabled;

        //Optional, used if you want to pool messages and reuse them when they are no longer in use.
        private Queue<Message> inactiveMessagePool = new Queue<Message>();

        public string TopicName { get; }

        public int QueueSize { get; }

        public bool Latch { get; }

        public ROSPublisher(string topicName, int queueSize, bool latch)
        {
            this.RosMessageName = MessageRegistry.GetRosMessageName<Message>();

            if (queueSize < 1)
            {
                throw new Exception("Queue size must be greater than or equal to 1.");
            }

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
            lock (outgoingMessages)
            {
                if (outgoingMessages.Count >= QueueSize)
                {
                    //Remove outgoing messages that don't fit in the queue.
                    //Recycle the message if applicable
                    RecycleMessageIfApplicable(outgoingMessages.First.Value);
                    //Update the overflow counter.
                    queueOverflowUnsentCounter++;
                    outgoingMessages.RemoveFirst();
                }

                //Add a new valid message to the end.
                outgoingMessages.AddLast(message);
            }
        }

        public void OnConnectionEstablished(MessageSerializer m_MessageSerializer, Stream stream)
        {
            //Register the publisher with the ROS Endpoint.
            RegisterPublisherIfApplicable(m_MessageSerializer, stream);
        }

        private SendToState RemoveMessageToSend(out Message messageToSend)
        {
            SendToState result = SendToState.NoMessageToSendError;
            messageToSend = null;
            lock (outgoingMessages)
            {
                if (queueOverflowUnsentCounter > 0)
                {
                    //This means that we can't send message to ROS as fast as we're generating them.
                    //This could potentially be bad as it means that we are dropping messages!
                    queueOverflowUnsentCounter--;
                    messageToSend = null;
                    result = SendToState.QueueFullWarning;
                }
                else if (outgoingMessages.Count > 0)
                {
                    //Retrieve the next message and populate messageToSend.
                    messageToSend = outgoingMessages.First.Value;
                    outgoingMessages.RemoveFirst();
                    result = SendToState.Normal;
                }
            }

            return result;
        }

        public bool PeekNextMessageToSend(out Message messageToSend)
        {
            bool result = false;
            messageToSend = null;
            lock (outgoingMessages)
            {
                if (outgoingMessages.Count > 0)
                {
                    messageToSend = outgoingMessages.First.Value;
                    result = true;
                }
            }


            return result;
        }

        private void SendMessageWithStream(MessageSerializer m_MessageSerializer, Stream stream, Message message)
        {
            RegisterPublisherIfApplicable(m_MessageSerializer, stream);

            //Clear the serializer
            m_MessageSerializer.Clear();
            //Prepare the data to send.
            m_MessageSerializer.Write(TopicName);
            m_MessageSerializer.SerializeMessageWithLength(message);
            //Send via the stream.
            m_MessageSerializer.SendTo(stream);
        }

        private void RegisterPublisherIfApplicable(MessageSerializer m_MessageSerializer, Stream stream)
        {
            if (PublisherRegistered)
            {
                return;
            }

            //Register the publisher before sending anything.
            SysCommandPublisherRegistration publisherRegistration =
                new SysCommandPublisherRegistration(this);
            publisherRegistration.SendTo(stream, m_MessageSerializer);
            PublisherRegistered = true;

            if (Latch && lastMessageSent != null)
            {
                //This topic is latching, so to mimic that functionality,
                //here the last sent message is sent again with the new connection.
                SendMessageWithStream(m_MessageSerializer, stream, lastMessageSent);
            }
        }

        public override SendToState SendInternal(MessageSerializer m_MessageSerializer, Stream stream)
        {
            RegisterPublisherIfApplicable(m_MessageSerializer, stream);
            SendToState sendToState = RemoveMessageToSend(out Message toSend);
            if (sendToState == SendToState.Normal)
            {
                SendMessageWithStream(m_MessageSerializer, stream, toSend);

                //Recycle the message (if applicable).
                if (Latch)
                {
                    if (lastMessageSent != null && lastMessageSent != toSend)
                    {
                        RecycleMessageIfApplicable(lastMessageSent);
                    }

                    lastMessageSent = toSend;
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
            lock (outgoingMessages)
            {
                toRecycle = new List<Message>(outgoingMessages);
                outgoingMessages.Clear();
                queueOverflowUnsentCounter = 0;
            }

            foreach (Message messageToRecycle in toRecycle)
            {
                RecycleMessageIfApplicable(messageToRecycle);
            }
        }

        private void RecycleMessageIfApplicable(Message toRecycle)
        {
            if (!MessagePoolEnabled)
            {
                return;
            }

            //Add the message back to the pool.
            AddMessageToPool(toRecycle);
        }

        #region Message Pooling

        public bool MessagePoolEnabled
        {
            get => _messagePoolEnabled;
            set
            {
                if (_messagePoolEnabled != value)
                {
                    _messagePoolEnabled = value;
                    if (!_messagePoolEnabled)
                    {
                        lock (inactiveMessagePool)
                        {
                            inactiveMessagePool.Clear();
                        }
                    }
                }
            }
        }

        public void AddMessageToPool(Message messageToRecycle)
        {
            Debug.Assert(MessagePoolEnabled,
                "Adding a message to a message pool that is not enabled, please set MessagePoolEnabled to true.");

            lock (inactiveMessagePool)
            {
                if (MessagePoolEnabled && inactiveMessagePool.Count < (QueueSize + 5))
                {
                    //Make sure we're only pooling a reasonable amount.
                    //We shouldn't need any more than the queue size plus a little.
                    inactiveMessagePool.Enqueue(messageToRecycle);
                }
            }
        }

        /**
         * @return a message of type T full of garbage data, be sure to update it accordingly.
         */
        public Message GetMessageFromPool()
        {
            Message result = null;
            MessagePoolEnabled = true;
            lock (inactiveMessagePool)
            {
                if (inactiveMessagePool.Count > 0)
                {
                    result = inactiveMessagePool.Dequeue();
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
