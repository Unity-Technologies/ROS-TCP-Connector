using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Tf2
{
    public class LookupTransformAction : Action<LookupTransformActionGoal, LookupTransformActionResult, LookupTransformActionFeedback, LookupTransformGoal, LookupTransformResult, LookupTransformFeedback>
    {
        public const string k_RosMessageName = "tf2_msgs/LookupTransformAction";
        public override string RosMessageName => k_RosMessageName;


        public LookupTransformAction() : base()
        {
            this.action_goal = new LookupTransformActionGoal();
            this.action_result = new LookupTransformActionResult();
            this.action_feedback = new LookupTransformActionFeedback();
        }

        public static LookupTransformAction Deserialize(MessageDeserializer deserializer) => new LookupTransformAction(deserializer);

        LookupTransformAction(MessageDeserializer deserializer)
        {
            this.action_goal = LookupTransformActionGoal.Deserialize(deserializer);
            this.action_result = LookupTransformActionResult.Deserialize(deserializer);
            this.action_feedback = LookupTransformActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
