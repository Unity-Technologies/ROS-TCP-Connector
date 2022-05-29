using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public interface IMessageDeserializer
    {
        T DeserializeMessage<T>(byte[] data) where T : Message;
        string DeserializeString(byte[] data);
    }
}
