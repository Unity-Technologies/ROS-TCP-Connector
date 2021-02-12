using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.ObjectRecognition
{
    public class ObjectRecognitionAction : Action<ObjectRecognitionActionGoal, ObjectRecognitionActionResult, ObjectRecognitionActionFeedback, ObjectRecognitionGoal, ObjectRecognitionResult, ObjectRecognitionFeedback>
    {
        public const string RosMessageName = "object_recognition_msgs-master/ObjectRecognitionAction";

        public ObjectRecognitionAction() : base()
        {
            this.action_goal = new ObjectRecognitionActionGoal();
            this.action_result = new ObjectRecognitionActionResult();
            this.action_feedback = new ObjectRecognitionActionFeedback();
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
