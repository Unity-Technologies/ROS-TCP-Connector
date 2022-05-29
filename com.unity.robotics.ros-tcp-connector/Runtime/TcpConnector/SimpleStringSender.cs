using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    /**
     * Simple implementation of a OutgoingMessageSender that is used for sys commands
     * as they are handled differently to typical ROS messages and sent as JSON strings.
     */
    public class SimpleStringSender : IConnectionTransport.ISendQueueItem
    {
        string command;
        string str;

        public SimpleStringSender(string command, string json)
        {
            this.command = command;
            this.str = json;
        }

        public IConnectionTransport.SendToState DoSend(IMessageSerializer m_MessageSerializer)
        {
            if (str != null)
            {
                m_MessageSerializer.SendString(command, str);
                return IConnectionTransport.SendToState.Normal;
            }
            return IConnectionTransport.SendToState.NoMessageToSendError;
        }

        public void ClearAllQueuedData()
        {
            str = null;
        }
    }
}
