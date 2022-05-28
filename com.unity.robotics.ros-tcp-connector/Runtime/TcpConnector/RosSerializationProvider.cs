using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class RosSerializationProvider : ISerializationProvider
    {
        public readonly bool m_IsRos2;

        public RosSerializationProvider(bool isRos2)
        {
            m_IsRos2 = isRos2;
        }

        public IMessageDeserializer CreateDeserializer()
        {
            return new MessageDeserializer(m_IsRos2);
        }

        public IMessageSerializer CreateSerializer(System.IO.Stream stream)
        {
            return new MessageSerializer(stream, m_IsRos2);
        }
    }
}
