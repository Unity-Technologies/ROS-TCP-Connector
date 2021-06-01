using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Nav
{
    public class MGetMapAction : Action<MGetMapActionGoal, MGetMapActionResult, MGetMapActionFeedback, MGetMapGoal, MGetMapResult, MGetMapFeedback>
    {
        public const string RosMessageName = "nav_msgs/GetMapAction";

        public MGetMapAction() : base()
        {
            this.action_goal = new MGetMapActionGoal();
            this.action_result = new MGetMapActionResult();
            this.action_feedback = new MGetMapActionFeedback();
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
