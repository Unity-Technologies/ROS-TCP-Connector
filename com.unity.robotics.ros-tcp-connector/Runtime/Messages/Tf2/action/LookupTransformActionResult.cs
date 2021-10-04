using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Tf2
{
    public class LookupTransformActionResult : ActionResult<LookupTransformResult>
    {
        public const string k_RosMessageName = "tf2_msgs/LookupTransformActionResult";
        public override string RosMessageName => k_RosMessageName;


        public LookupTransformActionResult() : base()
        {
            this.result = new LookupTransformResult();
        }

        public LookupTransformActionResult(HeaderMsg header, GoalStatusMsg status, LookupTransformResult result) : base(header, status)
        {
            this.result = result;
        }
        public static LookupTransformActionResult Deserialize(MessageDeserializer deserializer) => new LookupTransformActionResult(deserializer);

        LookupTransformActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = LookupTransformResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
