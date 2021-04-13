using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Tf2
{
    public class MLookupTransformActionResult : ActionResult<MLookupTransformResult>
    {
        public const string k_RosMessageName = "tf2_msgs/LookupTransformActionResult";
        public override string RosMessageName => k_RosMessageName;


        public MLookupTransformActionResult() : base()
        {
            this.result = new MLookupTransformResult();
        }

        public MLookupTransformActionResult(MHeader header, MGoalStatus status, MLookupTransformResult result) : base(header, status)
        {
            this.result = result;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(this.header.SerializationStatements());
            listOfSerializations.AddRange(this.status.SerializationStatements());
            listOfSerializations.AddRange(this.result.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.status.Deserialize(data, offset);
            offset = this.result.Deserialize(data, offset);

            return offset;
        }

    }
}
