using System;
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

        void Subscribe<T>(string topic, Action<T> callback) where T : Message;

        void Subscribe(string topic, string messageType, Action<Message> callback);
    }
}
