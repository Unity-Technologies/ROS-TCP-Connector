using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.MoveBase
{
    public class MoveBaseActionResult : ActionResult<MoveBaseResult>
    {
        public const string k_RosMessageName = "move_base_msgs/MoveBaseActionResult";
        public override string RosMessageName => k_RosMessageName;


        public MoveBaseActionResult() : base()
        {
            this.result = new MoveBaseResult();
        }

        public MoveBaseActionResult(HeaderMsg header, GoalStatusMsg status, MoveBaseResult result) : base(header, status)
        {
            this.result = result;
        }
        public static MoveBaseActionResult Deserialize(MessageDeserializer deserializer) => new MoveBaseActionResult(deserializer);

        MoveBaseActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = MoveBaseResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
