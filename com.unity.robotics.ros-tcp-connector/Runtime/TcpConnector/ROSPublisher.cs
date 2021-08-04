using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.ROSTCPConnector
{

    public abstract class ROSPublisherBase : SendsOutgoingMessages {
        public abstract string RosMessageName
        {
            get;
        }

        public string TopicName
        {
            get;
        }

        public int QueueSize
        {
            get;
        }

        public bool Latch
        {
            get;
        }

        public bool PublisherRegistered
        {
            get;
            set;
        }

        protected ROSPublisherBase(string topicName, int queueSize, bool latch)
        {
            if (queueSize < 1)
            {
                throw new Exception("Queue size must be greater than or equal to 1.");
            }
            TopicName = topicName;
            QueueSize = queueSize;
            Latch = latch;
            PublisherRegistered = false;
        }

        public abstract bool IsType(Type messageType);

        public bool EquivalentTo(string topicName, Type type, int? queueSize, bool? latch)
        {
            return TopicName == topicName && IsType(type) &&
                   (!queueSize.HasValue || QueueSize == queueSize) &&
                   (!latch.HasValue || Latch == latch);
        }

        public abstract void OnConnectionEstablished(MessageSerializer m_MessageSerializer, Stream stream);

    }

    public class ROSPublisher<T> : ROSPublisherBase where T : Message
    {

        public override string RosMessageName
        {
            get;
        }

        public override bool IsType(Type messageType)
        {
            return messageType == typeof(T);
        }

        //Messages waiting to be sent queue.
        private LinkedList<T> outgoingMessages = new LinkedList<T>();

        //Keeps track of how many outgoing messages were removed due to queue overflow.
        //If a message is published but the queue is full, the counter is incremented, the
        //first message in the queue is recycled and the message to publish is put on the end of the queue.
        //If this is non 0, a SendTo call will decrement the counter instead of sending data.
        private int queueOverflowUnsentCounter = 0;

        //Latching - This message will be set if latching is enabled, and on reconnection, it will be sent again.
        private T lastMessageSent = null;

        //Whether you want to pool messages for reuse (Used to reduce GC calls).
        private volatile bool _messagePoolEnabled;

        //Optional, used if you want to pool messages and reuse them when they are no longer in use.
        private Queue<T> inactiveMessagePool = new Queue<T>();

        public ROSPublisher(string topicName, int queueSize, bool latch) : base(topicName, queueSize, latch)
        {
            this.RosMessageName = MessageRegistry.GetRosMessageName<T>();
        }

        public void Publish(T message)
        {
            lock(outgoingMessages)
            {

                if (outgoingMessages.Count >= QueueSize)
                {
                    //Remove outgoing messages that don't fit in the queue.
                    //Recycle the message if applicable
                    RecycleMessage(outgoingMessages.First.Value);
                    //Update the overflow counter.
                    queueOverflowUnsentCounter++;
                    outgoingMessages.RemoveFirst();
                }

                //Add a new valid message to the end.
                outgoingMessages.AddLast(message);
            }
        }

        public override void OnConnectionEstablished(MessageSerializer m_MessageSerializer, Stream stream)
        {
            //Register the publisher with the ROS Endpoint.
            RegisterPublisherIfApplicable(m_MessageSerializer, stream);

        }

        private SendToState RemoveMessageToSend(out T messageToSend)
        {
            SendToState result = SendToState.NoMessageToSendError;
            messageToSend = null;
            lock(outgoingMessages)
            {
                if (queueOverflowUnsentCounter > 0)
                {
                    //This means that we can't send message to ROS as fast as we're generating them.
                    //This could potentially be bad as it means that we are dropping messages!
                    queueOverflowUnsentCounter--;
                    messageToSend = null;
                    result = SendToState.QueueFullWarning;
                } else if (outgoingMessages.Count > 0)
                {
                    //Retrieve the next message and populate messageToSend.
                    messageToSend = outgoingMessages.First.Value;
                    outgoingMessages.RemoveFirst();
                    result = SendToState.Normal;
                }
            }

            return result;
        }

        public bool PeekNextMessageToSend(out T messageToSend)
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
            SendToState sendToState = RemoveMessageToSend(out T toSend);
            if (sendToState == SendToState.Normal)
            {
                SendMessageWithStream(m_MessageSerializer, stream, toSend);

                //Recycle the message (if applicable).
                if (Latch)
                {
                    if (lastMessageSent != null && lastMessageSent != toSend)
                    {
                        RecycleMessage(lastMessageSent);
                    }
                    lastMessageSent = toSend;
                }
                else
                {
                    RecycleMessage(toSend);
                }
            }
            return sendToState;
        }

        public override void ClearAllQueuedData()
        {

            List<T> toRecycle;
            lock(outgoingMessages)
            {
                toRecycle = new List<T>(outgoingMessages);
                outgoingMessages.Clear();
                queueOverflowUnsentCounter = 0;
            }

            foreach (T messageToRecycle in toRecycle)
            {
                RecycleMessage(messageToRecycle);
            }
        }

        private void RecycleMessage(T toRecycle)
        {
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
                        lock(inactiveMessagePool)
                        {
                            inactiveMessagePool.Clear();
                        }
                    }
                }

            }
        }

        public void AddMessageToPool(T messageToRecycle)
        {
            if (!MessagePoolEnabled)
            {
                //No message pooling, let the GC handle this message.
                return;
            }
            lock(inactiveMessagePool)
            {
                if (inactiveMessagePool.Count < (QueueSize + 5))
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
        public T GetMessageFromPool()
        {
            T result = null;
            MessagePoolEnabled = true;
            lock(inactiveMessagePool)
            {
                if (inactiveMessagePool.Count > 0)
                {
                    result = inactiveMessagePool.Dequeue();
                }
            }

            if (result == null)
            {
                result = Activator.CreateInstance<T>();
            }

            return result;
        }

        #endregion

    }
}
