using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public interface IMessageSerializer
    {
        byte[] SerializeMessage(string topic, Message msg);
        void SendMessage(string topic, Message msg, System.IO.Stream outStream);
    }
}
