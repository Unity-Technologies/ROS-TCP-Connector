using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IConnectionConsumer
    {
        bool ConnectOnStart { get; }
        IConnectionTransport ConnectionTransport { get; set; }
        ISerializationProvider SerializationProvider { get; }
        void OnConnectionStartedCallback(IMessageSerializer serializer);
        void OnConnectionLostCallback();
    }
}
