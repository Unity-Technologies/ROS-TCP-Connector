using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface ISendQueueItem
    {
        public enum SendToState
        {
            Normal,
            NoMessageToSendError,
            QueueFullWarning
        }

        SendToState DoSend(IMessageSerializer m_MessageSerializer);

        void ClearAllQueuedData();
    }

    /**
     * Simple implementation of a OutgoingMessageSender that is used for sys commands
     * as they are handled differently to typical ROS messages and sent as JSON strings.
     */
    public class StringSender : ISendQueueItem
    {
        string command;
        string str;

        public StringSender(string command, string json)
        {
            this.command = command;
            this.str = json;
        }

        public ISendQueueItem.SendToState DoSend(IMessageSerializer m_MessageSerializer)
        {
            if (str != null)
            {
                m_MessageSerializer.SendString(command, str);
                return ISendQueueItem.SendToState.Normal;
            }
            return ISendQueueItem.SendToState.NoMessageToSendError;
        }

        public void ClearAllQueuedData()
        {
            str = null;
        }
    }

    public class SimpleMessageSender : ISendQueueItem
    {
        string command;
        Message msg;

        public SimpleMessageSender(string command, Message msg)
        {
            this.command = command;
            this.msg = msg;
        }

        public ISendQueueItem.SendToState DoSend(IMessageSerializer m_MessageSerializer)
        {
            if (msg != null)
            {
                m_MessageSerializer.SendMessage(command, msg);
                return ISendQueueItem.SendToState.Normal;
            }
            return ISendQueueItem.SendToState.NoMessageToSendError;
        }

        public void ClearAllQueuedData()
        {
            msg = null;
        }
    }
}
