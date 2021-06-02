//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class MPoint : Message
    {
        public const string k_RosMessageName = "geometry_msgs/Point";
        public override string RosMessageName => k_RosMessageName;

        //  This contains the position of a point in free space
        public double x;
        public double y;
        public double z;

        public MPoint()
        {
            this.x = 0.0;
            this.y = 0.0;
            this.z = 0.0;
        }

        public MPoint(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.x));
            listOfSerializations.Add(BitConverter.GetBytes(this.y));
            listOfSerializations.Add(BitConverter.GetBytes(this.z));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.x = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.y = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.z = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

        public override string ToString()
        {
            return "MPoint: " +
            "\nx: " + x.ToString() +
            "\ny: " + y.ToString() +
            "\nz: " + z.ToString();
        }

        [UnityEngine.RuntimeInitializeOnLoadMethod(UnityEngine.RuntimeInitializeLoadType.AfterAssembliesLoaded)]
        static void OnLoad()
        {
            MessageRegistry.Register<MPoint>(k_RosMessageName);
        }
    }
}