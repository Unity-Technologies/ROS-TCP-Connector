//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Nav
{
    public class MGetMapResult : Message
    {
        public const string RosMessageName = "nav_msgs/GetMap";

        public MOccupancyGrid map;

        public MGetMapResult()
        {
            this.map = new MOccupancyGrid();
        }

        public MGetMapResult(MOccupancyGrid map)
        {
            this.map = map;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(map.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.map.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MGetMapResult: " +
            "\nmap: " + map.ToString();
        }
    }
}
