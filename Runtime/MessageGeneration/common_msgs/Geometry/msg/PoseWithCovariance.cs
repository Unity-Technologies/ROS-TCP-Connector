using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class PoseWithCovariance : Message
    {
        public const string RosMessageName = "geometry_msgs/PoseWithCovariance";

        //  This represents a pose in free space with uncertainty.
        public Pose pose { get; set; }
        //  Row-major representation of the 6x6 covariance matrix
        //  The orientation parameters use a fixed-axis representation.
        //  In order, the parameters are:
        //  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        public double[] covariance { get; set; }

        public PoseWithCovariance()
        {
            this.pose = new Pose();
            this.covariance = new double[36];
        }

        public PoseWithCovariance(Pose pose, double[] covariance)
        {
            this.pose = pose;
            this.covariance = covariance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(pose.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(covariance.Length));
            foreach(var entry in covariance)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.pose.Deserialize(data, offset);
            
            var covarianceArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.covariance= new double[covarianceArrayLength];
            for(var i =0; i <covarianceArrayLength; i++)
            {
                this.covariance[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }

            return offset;
        }

    }
}