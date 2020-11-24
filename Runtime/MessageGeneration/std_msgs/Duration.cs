using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Std
{
    public class Duration : Message
    {
        public const string RosMessageName = "std_msgs/Duration";

        public Duration data;

        public Duration()
        {
            this.data = new Duration();
        }

        public Duration(Duration data)
        {
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(data.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.data.Deserialize(data, offset);

            return offset;
        }

    }
}
