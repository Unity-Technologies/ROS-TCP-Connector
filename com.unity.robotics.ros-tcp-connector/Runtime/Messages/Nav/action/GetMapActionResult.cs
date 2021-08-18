using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Nav
{
    public class GetMapActionResult : ActionResult<GetMapResult>
    {
        public const string k_RosMessageName = "nav_msgs/GetMapActionResult";
        public override string RosMessageName => k_RosMessageName;


        public GetMapActionResult() : base()
        {
            this.result = new GetMapResult();
        }

        public GetMapActionResult(HeaderMsg header, GoalStatusMsg status, GetMapResult result) : base(header, status)
        {
            this.result = result;
        }
        public static GetMapActionResult Deserialize(MessageDeserializer deserializer) => new GetMapActionResult(deserializer);

        GetMapActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = GetMapResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
