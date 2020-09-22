/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Byte : Message
    {
        public const string RosMessageName = "std_msgs/Byte";

        public sbyte data { get; set; }

        public Byte()
        {
            this.data = 0;
        }

        public Byte(sbyte data)
        {
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.data));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.data = (sbyte)data[offset];;
            offset += 1;

            return offset;
        }

    }
}
