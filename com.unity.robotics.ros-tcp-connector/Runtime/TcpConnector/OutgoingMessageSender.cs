using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.ROSTCPConnector
{
    public abstract class OutgoingMessageSender
    {

        public enum SendToState
        {
            Normal,
            NoMessageToSendError,
            QueueFullWarning
        }

        public abstract SendToState SendInternal(MessageSerializer m_MessageSerializer, System.IO.Stream stream);

        public abstract void ClearAllQueuedData();
    }

    /**
     * Simple implementation of a OutgoingMessageSender that is used for sys commands
     * as they are handled differently to typical ROS messages and sent as JSON strings.
     */
    public class SysCommandSender : OutgoingMessageSender
    {
        List<byte[]> m_ListOfSerializations;

        public SysCommandSender(List<byte[]> listOfSerializations)
        {
            m_ListOfSerializations = listOfSerializations;
        }

        public override SendToState SendInternal(MessageSerializer m_MessageSerializer, Stream stream)
        {
            foreach (byte[] statement in m_ListOfSerializations)
            {
                stream.Write(statement, 0, statement.Length);
            }

            return SendToState.Normal;
        }

        public override void ClearAllQueuedData()
        {
            m_ListOfSerializations.Clear();
        }
    }
}
