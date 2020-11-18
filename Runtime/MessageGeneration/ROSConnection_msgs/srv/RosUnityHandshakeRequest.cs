using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.TcpEndpoint
{
    public class RosUnityHandshakeRequest : Message
    {
        public const string RosMessageName = "tcp_endpoint/RosUnityHandshake";

        public string ip;
        public ushort port;

        public RosUnityHandshakeRequest()
        {
            this.ip = "";
            this.port = 0;
        }

        public RosUnityHandshakeRequest(string ip, ushort port)
        {
            this.ip = ip;
            this.port = port;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.ip));
            listOfSerializations.Add(BitConverter.GetBytes(this.port));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var ipStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.ip = DeserializeString(data, offset, ipStringBytesLength);
            offset += ipStringBytesLength;
            this.port = BitConverter.ToUInt16(data, offset);
            offset += 2;

            return offset;
        }

    }
}
