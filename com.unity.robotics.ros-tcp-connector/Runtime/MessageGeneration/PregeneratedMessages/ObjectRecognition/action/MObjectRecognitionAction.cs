using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.ObjectRecognition
{
    public class MObjectRecognitionAction : Action<MObjectRecognitionActionGoal, MObjectRecognitionActionResult, MObjectRecognitionActionFeedback, MObjectRecognitionGoal, MObjectRecognitionResult, MObjectRecognitionFeedback>
    {
        public const string RosMessageName = "object_recognition_msgs-master/ObjectRecognitionAction";

        public MObjectRecognitionAction() : base()
        {
            this.action_goal = new MObjectRecognitionActionGoal();
            this.action_result = new MObjectRecognitionActionResult();
            this.action_feedback = new MObjectRecognitionActionFeedback();
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
