using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public interface IMessageSerializer
    {
        void SendMessage(string topic, Message msg, System.IO.Stream outStream);
        void SendString(string topic, string str, System.IO.Stream outStream);
    }
}
