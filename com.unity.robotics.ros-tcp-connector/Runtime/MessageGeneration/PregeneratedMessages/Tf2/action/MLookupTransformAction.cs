using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.Tf2
{
    public class MLookupTransformAction : Action<MLookupTransformActionGoal, MLookupTransformActionResult, MLookupTransformActionFeedback, MLookupTransformGoal, MLookupTransformResult, MLookupTransformFeedback>
    {
        public const string RosMessageName = "tf2_msgs/LookupTransformAction";

        public MLookupTransformAction() : base()
        {
            this.action_goal = new MLookupTransformActionGoal();
            this.action_result = new MLookupTransformActionResult();
            this.action_feedback = new MLookupTransformActionFeedback();
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
