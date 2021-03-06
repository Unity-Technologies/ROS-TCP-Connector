//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ObjectRecognition
{
    public class MObjectInformation : Message
    {
        public const string RosMessageName = "object_recognition_msgs/ObjectInformation";

        // ############################################# VISUALIZATION INFO ######################################################
        // ################## THIS INFO SHOULD BE OBTAINED INDEPENDENTLY FROM THE CORE, LIKE IN AN RVIZ PLUGIN ###################
        //  The human readable name of the object
        public string name;
        //  The full mesh of the object: this can be useful for display purposes, augmented reality ... but it can be big
        //  Make sure the type is MESH
        public Shape.MMesh ground_truth_mesh;
        //  Sometimes, you only have a cloud in the DB
        //  Make sure the type is POINTS
        public Sensor.MPointCloud2 ground_truth_point_cloud;

        public MObjectInformation()
        {
            this.name = "";
            this.ground_truth_mesh = new Shape.MMesh();
            this.ground_truth_point_cloud = new Sensor.MPointCloud2();
        }

        public MObjectInformation(string name, Shape.MMesh ground_truth_mesh, Sensor.MPointCloud2 ground_truth_point_cloud)
        {
            this.name = name;
            this.ground_truth_mesh = ground_truth_mesh;
            this.ground_truth_point_cloud = ground_truth_point_cloud;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.AddRange(ground_truth_mesh.SerializationStatements());
            listOfSerializations.AddRange(ground_truth_point_cloud.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            offset = this.ground_truth_mesh.Deserialize(data, offset);
            offset = this.ground_truth_point_cloud.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MObjectInformation: " +
            "\nname: " + name.ToString() +
            "\nground_truth_mesh: " + ground_truth_mesh.ToString() +
            "\nground_truth_point_cloud: " + ground_truth_point_cloud.ToString();
        }
    }
}
