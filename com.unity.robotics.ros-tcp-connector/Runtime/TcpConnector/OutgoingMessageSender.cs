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

    public class SimpleDataSender : OutgoingMessageSender
    {
        private List<byte[]> m_ListOfSerializations;

        public SimpleDataSender(List<byte[]> m_ListOfSerializations)
        {
            this.m_ListOfSerializations = m_ListOfSerializations;
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
