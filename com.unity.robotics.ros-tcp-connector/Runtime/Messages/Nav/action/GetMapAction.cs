using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Nav
{
    public class GetMapAction : Action<GetMapActionGoal, GetMapActionResult, GetMapActionFeedback, GetMapGoal, GetMapResult, GetMapFeedback>
    {
        public const string k_RosMessageName = "nav_msgs/GetMapAction";
        public override string RosMessageName => k_RosMessageName;


        public GetMapAction() : base()
        {
            this.action_goal = new GetMapActionGoal();
            this.action_result = new GetMapActionResult();
            this.action_feedback = new GetMapActionFeedback();
        }

        public static GetMapAction Deserialize(MessageDeserializer deserializer) => new GetMapAction(deserializer);

        GetMapAction(MessageDeserializer deserializer)
        {
            this.action_goal = GetMapActionGoal.Deserialize(deserializer);
            this.action_result = GetMapActionResult.Deserialize(deserializer);
            this.action_feedback = GetMapActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
