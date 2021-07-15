using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.ObjectRecognition
{
    public class ObjectRecognitionAction : Action<ObjectRecognitionActionGoal, ObjectRecognitionActionResult, ObjectRecognitionActionFeedback, ObjectRecognitionGoal, ObjectRecognitionResult, ObjectRecognitionFeedback>
    {
        public const string k_RosMessageName = "object_recognition_msgs-master/ObjectRecognitionAction";
        public override string RosMessageName => k_RosMessageName;


        public ObjectRecognitionAction() : base()
        {
            this.action_goal = new ObjectRecognitionActionGoal();
            this.action_result = new ObjectRecognitionActionResult();
            this.action_feedback = new ObjectRecognitionActionFeedback();
        }

        public static ObjectRecognitionAction Deserialize(MessageDeserializer deserializer) => new ObjectRecognitionAction(deserializer);

        ObjectRecognitionAction(MessageDeserializer deserializer)
        {
            this.action_goal = ObjectRecognitionActionGoal.Deserialize(deserializer);
            this.action_result = ObjectRecognitionActionResult.Deserialize(deserializer);
            this.action_feedback = ObjectRecognitionActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
