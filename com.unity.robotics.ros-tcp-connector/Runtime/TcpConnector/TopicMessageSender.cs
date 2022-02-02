using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class TopicMessageSender : OutgoingMessageSender
    {
        public string RosMessageName { get; private set; }

        public string TopicName { get; private set; }

        public int QueueSize { get; private set; }

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
        IMessagePool m_MessagePool;
        public bool MessagePoolEnabled => m_MessagePool != null;

        public TopicMessageSender(string topicName, string rosMessageName, int queueSize)
        {
            if (queueSize < 1)
            {
                throw new Exception("Queue size must be greater than or equal to 1.");
            }

            TopicName = topicName;
            RosMessageName = rosMessageName;
            QueueSize = queueSize;
        }

        internal void Queue(Message message)
        {
            lock (m_OutgoingMessages)
            {
                if (m_OutgoingMessages.Count >= QueueSize)
                {
                    //Remove outgoing messages that don't fit in the queue.
                    //Recycle the message if applicable
                    TryRecycleMessage(m_OutgoingMessages.First.Value);
                    //Update the overflow counter.
                    m_QueueOverflowUnsentCounter++;
                    m_OutgoingMessages.RemoveFirst();
                }

                //Add a new valid message to the end.
                m_OutgoingMessages.AddLast(message);
            }
        }

        SendToState GetMessageToSend(out Message messageToSend)
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
            //Clear the serializer
            messageSerializer.Clear();
            //Prepare the data to send.
            messageSerializer.Write(TopicName);
            messageSerializer.SerializeMessageWithLength(message);
            //Send via the stream.
            messageSerializer.SendTo(stream);
        }

        public void PrepareLatchMessage()
        {
            if (m_LastMessageSent != null && !m_OutgoingMessages.Any())
            {
                //This topic is latching, so to mimic that functionality,
                // the last sent message is sent again with the new connection.
                m_OutgoingMessages.AddFirst(m_LastMessageSent);
            }
        }

        public override SendToState SendInternal(MessageSerializer messageSerializer, Stream stream)
        {
            SendToState sendToState = GetMessageToSend(out Message toSend);
            if (sendToState == SendToState.Normal)
            {
                SendMessageWithStream(messageSerializer, stream, toSend);

                //Recycle the message (if applicable).
                if (m_LastMessageSent != null && m_LastMessageSent != toSend)
                {
                    TryRecycleMessage(m_LastMessageSent);
                }

                m_LastMessageSent = toSend;
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
                TryRecycleMessage(messageToRecycle);
            }
        }

        void TryRecycleMessage(Message toRecycle)
        {
            if (m_MessagePool != null)
            {
                //Add the message back to the pool.
                m_MessagePool.AddMessage(toRecycle);
            }
        }

        public void SetMessagePool(IMessagePool messagePool)
        {
            m_MessagePool = messagePool;
        }
    }
}
