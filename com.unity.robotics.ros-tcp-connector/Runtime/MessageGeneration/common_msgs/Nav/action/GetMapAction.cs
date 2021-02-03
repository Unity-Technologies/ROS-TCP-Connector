using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Nav
{
    public class GetMapAction : Action<GetMapActionGoal, GetMapActionResult, GetMapActionFeedback, GetMapGoal, GetMapResult, GetMapFeedback>
    {
        public const string RosMessageName = "nav_msgs/GetMapAction";

        public GetMapAction() : base()
        {
            this.action_goal = new GetMapActionGoal();
            this.action_result = new GetMapActionResult();
            this.action_feedback = new GetMapActionFeedback();
        }

        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(this.action_goal.SerializationStatements());
            listOfSerializations.AddRange(this.action_result.SerializationStatements());
            listOfSerializations.AddRange(this.action_feedback.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.action_goal.Deserialize(data, offset);
            offset = this.action_result.Deserialize(data, offset);
            offset = this.action_feedback.Deserialize(data, offset);

            return offset;
        }

    }
}
