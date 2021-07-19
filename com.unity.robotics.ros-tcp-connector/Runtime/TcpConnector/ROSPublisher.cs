using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.ROSTCPConnector
{

    public class PublishedMessage<T> where T : Message
    {
        public bool valid;
        public T message;

        public PublishedMessage(T message)
        {
            this.valid = true;
            this.message = message;
        }

        public void Invalidate()
        {
            this.valid = false;
            this.message = null;
        }
    }

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
            TopicName = topicName;
            QueueSize = queueSize;
            Latch = latch;
        }

        public abstract bool IsType(Type messageType);

        public bool EquivalentTo(string topicName, Type type, int queueSize, bool latch)
        {
            return TopicName == topicName && IsType(type) && QueueSize == queueSize && Latch == latch;
        }

        public bool EquivalentTo(string topicName, string rosMessageName, int queueSize, bool latch)
        {
            return TopicName == topicName && RosMessageName == rosMessageName && QueueSize == queueSize && Latch == latch;
        }

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

        //Messages waiting to be sent.
        //TODO - I don't like how this is implemented, I think it's confusing and could be done better.
        private LinkedList<PublishedMessage<T>> outgoingMessages = new LinkedList<PublishedMessage<T>>();
        private LinkedListNode<PublishedMessage<T>> nextValidOutgoingMessage = null;
        private int validOutgoingMessageCount = 0;


        //Whether you want to pool messages for reuse (Used to reduce GC calls).
        private volatile bool messagePoolEnabled;

        //Optional, used if you want to pool messages and reuse them when they are no longer in use.
        private Queue<T> inactiveMessagePool = new Queue<T>();

        public ROSPublisher(string topicName, int queueSize, bool latch) : base(topicName, queueSize, latch)
        {
            this.RosMessageName = MessageRegistry.GetRosMessageName<T>();
            this.PublisherRegistered = false;
        }

        public void Send(T message)
        {
            lock(outgoingMessages)
            {

                if (validOutgoingMessageCount >= QueueSize)
                {
                    //Remove outgoing messages that don't fit in the queue.
                    validOutgoingMessageCount--;
                    if (nextValidOutgoingMessage == null || nextValidOutgoingMessage.List == null)
                    {
                        //The next valid outgoing message is no longer part of the list (probably because it was sent).
                        //Set the next valid message to the beginning of the list.
                        nextValidOutgoingMessage = outgoingMessages.First;
                    }

                    if (nextValidOutgoingMessage != null)
                    {
                        //Recycle the message if applicable
                        AddMessageToPool(nextValidOutgoingMessage.Value.message);
                        //Flag that the message shouldn't be sent as it was removed from the queue.
                        nextValidOutgoingMessage.Value.Invalidate();
                        nextValidOutgoingMessage = nextValidOutgoingMessage.Next;
                    }
                }

                validOutgoingMessageCount++;
                outgoingMessages.AddLast(new PublishedMessage<T>(message));
                if (nextValidOutgoingMessage == null || nextValidOutgoingMessage.List == null)
                {
                    //The next valid outgoing message is no longer part of the list (probably because it was sent).
                    //Set the next valid message to the beginning of the list.
                    nextValidOutgoingMessage = outgoingMessages.First;
                }
            }
        }

        public bool RemoveMessageToSend(out PublishedMessage<T> messageToSend)
        {
            bool result = false;
            messageToSend = null;
            lock(outgoingMessages)
            {
                if (outgoingMessages.Count > 0)
                {
                    messageToSend = outgoingMessages.First.Value;
                    outgoingMessages.RemoveFirst();

                    if (nextValidOutgoingMessage.List == null)
                    {
                        validOutgoingMessageCount--;
                    }

                    result = true;
                }
            }

            return result;
        }

        public bool PeekNextMessageToSend(out PublishedMessage<T> messageToSend)
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

        public override SendToState SendTo(MessageSerializer m_MessageSerializer, Stream stream)
        {
            if (RemoveMessageToSend(out PublishedMessage<T> toSend))
            {
                if (toSend.valid)
                {

                    if (!PublisherRegistered)
                    {
                        //Register the publisher before sending anything.
                        SysCommandPublisherRegistration publisherRegistration =
                            new SysCommandPublisherRegistration(this);
                        publisherRegistration.SendTo(stream, m_MessageSerializer);
                        PublisherRegistered = true;
                    }

                    //Clear the serializer
                    m_MessageSerializer.Clear();
                    //Prepare the data to send.
                    m_MessageSerializer.Write(TopicName);
                    m_MessageSerializer.SerializeMessageWithLength(toSend.message);
                    //Send via the stream.
                    m_MessageSerializer.SendTo(stream);
                    //Recycle the message (if applicable).
                    AddMessageToPool(toSend.message);
                    toSend.Invalidate();
                    return SendToState.Normal;
                }
                //This means that we can't send message to ROS as fast as we're generating them.
                //This could potentially be bad as it means that we are dropping messages!
                return SendToState.QueueFullWarning;
            }
            //This shouldn't happen, it's just used here as a sanity check.
            return SendToState.NoMessageToSendError;
        }

        #region Message Pooling

        public void SetMessagePoolEnabled(bool messagePoolEnabled)
        {
            this.messagePoolEnabled = messagePoolEnabled;
            if (!messagePoolEnabled)
            {
                lock(inactiveMessagePool)
                {
                    inactiveMessagePool.Clear();
                }
            }
        }

        public void AddMessageToPool(T messageToRecycle)
        {
            if (!messagePoolEnabled)
            {
                //No message pooling, let the GC handle this message.
                return;
            }
            lock(inactiveMessagePool)
            {
                if (inactiveMessagePool.Count < (QueueSize + 2))
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
