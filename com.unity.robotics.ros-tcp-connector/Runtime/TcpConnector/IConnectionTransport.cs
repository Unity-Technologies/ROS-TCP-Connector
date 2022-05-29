using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IConnectionTransport
    {
        public interface ISendQueueItem
        {
            SendToState DoSend(IMessageSerializer m_MessageSerializer);

            void ClearAllQueuedData();
        }

        public enum SendToState
        {
            Normal,
            NoMessageToSendError,
            QueueFullWarning
        }

        void Connect();
        void Disconnect();

        bool HasConnection { get; }
        bool HasConnectionError { get; }

        bool TryRead(out string topic, out byte[] data);

        void Send(ISendQueueItem sender);
    }
}
