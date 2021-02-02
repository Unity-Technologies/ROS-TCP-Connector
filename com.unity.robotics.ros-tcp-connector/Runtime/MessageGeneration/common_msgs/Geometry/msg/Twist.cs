//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Twist : Message
    {
        public const string RosMessageName = "geometry_msgs/Twist";

        //  This expresses velocity in free space broken into its linear and angular parts.
        public Vector3 linear;
        public Vector3 angular;

        public Twist()
        {
            this.linear = new Vector3();
            this.angular = new Vector3();
        }

        public Twist(Vector3 linear, Vector3 angular)
        {
            this.linear = linear;
            this.angular = angular;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(linear.SerializationStatements());
            listOfSerializations.AddRange(angular.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.linear.Deserialize(data, offset);
            offset = this.angular.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "Twist: " +
            "\nlinear: " + linear.ToString() +
            "\nangular: " + angular.ToString();
        }
    }
}