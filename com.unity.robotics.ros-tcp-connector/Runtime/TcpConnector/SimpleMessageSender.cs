using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.ROSTCPConnector
{
    public class SimpleMessageSender : IConnectionTransport.ISendQueueItem
    {
        string command;
        Message msg;

        public SimpleMessageSender(string command, Message msg)
        {
            this.command = command;
            this.msg = msg;
        }

        public IConnectionTransport.SendToState DoSend(IMessageSerializer m_MessageSerializer)
        {
            if (msg != null)
            {
                m_MessageSerializer.SendMessage(command, msg);
                return IConnectionTransport.SendToState.Normal;
            }
            return IConnectionTransport.SendToState.NoMessageToSendError;
        }

        public void ClearAllQueuedData()
        {
            msg = null;
        }
    }
}
