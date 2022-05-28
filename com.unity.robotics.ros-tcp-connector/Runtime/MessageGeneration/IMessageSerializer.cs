using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public interface IMessageSerializer
    {
        void SendMessage(string topic, Message msg);
        void SendString(string topic, string str);
    }
}
