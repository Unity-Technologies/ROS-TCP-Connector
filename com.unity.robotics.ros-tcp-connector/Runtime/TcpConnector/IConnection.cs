using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IConnection
    {
        void Connect();
        void Disconnect();

        IPublisher RegisterPublisher<T>(string topic) where T : Message;
        IPublisher RegisterPublisher(string topic, string type);
    }
}
