using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public class Message
    {
        public virtual void SerializeTo(MessageSerializer state)
        {
            throw new NotImplementedException();
        }

        // The following are for backwards compatibility only:
        public virtual List<byte[]> SerializationStatements() => throw new NotImplementedException();
        public virtual int Deserialize(byte[] data, int offset) => throw new NotImplementedException();
        public byte[] Serialize() => throw new NotImplementedException();
        public byte[] SerializeString(string s) => throw new NotImplementedException();
        public int DeserializeLength(byte[] data, int offset) => throw new NotImplementedException();
        public string DeserializeString(byte[] data, int offset, int length) => throw new NotImplementedException();
    }
}