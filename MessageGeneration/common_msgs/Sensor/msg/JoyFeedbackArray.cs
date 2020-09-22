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

namespace RosMessageTypes.Sensor
{
    public class JoyFeedbackArray : Message
    {
        public const string RosMessageName = "sensor_msgs/JoyFeedbackArray";

        //  This message publishes values for multiple feedback at once. 
        public JoyFeedback[] array { get; set; }

        public JoyFeedbackArray()
        {
            this.array = new JoyFeedback[0];
        }

        public JoyFeedbackArray(JoyFeedback[] array)
        {
            this.array = array;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(array.Length));
            foreach(var entry in array)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var arrayArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.array= new JoyFeedback[arrayArrayLength];
            for(var i =0; i <arrayArrayLength; i++)
            {
                this.array[i] = new JoyFeedback();
                offset = this.array[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
