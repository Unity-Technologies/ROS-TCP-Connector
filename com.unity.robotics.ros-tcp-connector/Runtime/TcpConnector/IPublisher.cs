using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IPublisher
    {
        string Topic { get; }
        string MessageType { get; }
        void Publish(Message msg);
    }
}
