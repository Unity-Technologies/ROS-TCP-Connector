//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ObjectRecognition
{
    public class GetObjectInformationRequest : Message
    {
        public const string RosMessageName = "object_recognition_msgs-master/GetObjectInformation";

        //  Retrieve extra data from the DB for a given object
        //  The type of the object to retrieve info from
        public ObjectType type;

        public GetObjectInformationRequest()
        {
            this.type = new ObjectType();
        }

        public GetObjectInformationRequest(ObjectType type)
        {
            this.type = type;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(type.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.type.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "GetObjectInformationRequest: " +
            "\ntype: " + type.ToString();
        }
    }
}