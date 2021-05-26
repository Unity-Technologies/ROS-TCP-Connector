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

        [Obsolete("Message needs to be regenerated")]
        public virtual List<byte[]> SerializationStatements() => throw new NotImplementedException();

        [Obsolete("Message needs to be regenerated")]
        public virtual int Deserialize(byte[] data, int offset) => throw new NotImplementedException();

        [Obsolete("Message needs to be regenerated")]
        public byte[] SerializeString(string s) => throw new NotImplementedException();

        [Obsolete("Message needs to be regenerated")]
        public int DeserializeLength(byte[] data, int offset) => throw new NotImplementedException();

        [Obsolete("Message needs to be regenerated")]
        public string DeserializeString(byte[] data, int offset, int length) => throw new NotImplementedException();
    }
}