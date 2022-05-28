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
        void Init(
            ISerializationProvider serializationProvider,
            Action<Stream> onConnectionStartedCallback,
            Action onConnectionLostCallback);

        void Connect();
        void Disconnect();

        bool HasConnection { get; }
        bool HasConnectionError { get; }

        bool TryRead(out string topic, out byte[] data);

        void Send(string topic, string text);
        void Send(string topic, Message msg);
        void Send(IOutgoingMessageSender sender);
    }
}
