using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IOutgoingMessageSender
    {
        public enum SendToState
        {
            Normal,
            NoMessageToSendError,
            QueueFullWarning
        }

        SendToState SendInternal(IMessageSerializer m_MessageSerializer, Stream stream);

        void ClearAllQueuedData();
    }

    /**
     * Simple implementation of a OutgoingMessageSender that is used for sys commands
     * as they are handled differently to typical ROS messages and sent as JSON strings.
     */
    public class StringSender : IOutgoingMessageSender
    {
        string command;
        string str;

        public StringSender(string command, string json)
        {
            this.command = command;
            this.str = json;
        }

        public IOutgoingMessageSender.SendToState SendInternal(IMessageSerializer m_MessageSerializer, Stream stream)
        {
            if (str != null)
            {
                m_MessageSerializer.SendString(command, str, stream);
            }
            return IOutgoingMessageSender.SendToState.Normal;
        }

        public void ClearAllQueuedData()
        {
            str = null;
        }
    }

    public class SimpleMessageSender : IOutgoingMessageSender
    {
        string command;
        Message msg;

        public SimpleMessageSender(string command, Message msg)
        {
            this.command = command;
            this.msg = msg;
        }

        public IOutgoingMessageSender.SendToState SendInternal(IMessageSerializer m_MessageSerializer, Stream stream)
        {
            if (msg != null)
            {
                m_MessageSerializer.SendMessage(command, msg, stream);
            }
            return IOutgoingMessageSender.SendToState.Normal;
        }

        public void ClearAllQueuedData()
        {
            msg = null;
        }
    }
}
