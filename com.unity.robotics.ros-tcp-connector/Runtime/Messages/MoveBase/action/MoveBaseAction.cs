using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.MoveBase
{
    public class MoveBaseAction : Action<MoveBaseActionGoal, MoveBaseActionResult, MoveBaseActionFeedback, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback>
    {
        public const string k_RosMessageName = "move_base_msgs/MoveBaseAction";


        public MoveBaseAction() : base()
        {
            this.action_goal = new MoveBaseActionGoal();
            this.action_result = new MoveBaseActionResult();
            this.action_feedback = new MoveBaseActionFeedback();
        }

        public static MoveBaseAction Deserialize(MessageDeserializer deserializer) => new MoveBaseAction(deserializer);

        MoveBaseAction(MessageDeserializer deserializer)
        {
            this.action_goal = MoveBaseActionGoal.Deserialize(deserializer);
            this.action_result = MoveBaseActionResult.Deserialize(deserializer);
            this.action_feedback = MoveBaseActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
