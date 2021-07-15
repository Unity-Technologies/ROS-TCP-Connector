using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.ObjectRecognition
{
    public class ObjectRecognitionActionFeedback : ActionFeedback<ObjectRecognitionFeedback>
    {
        public const string k_RosMessageName = "object_recognition_msgs-master/ObjectRecognitionActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public ObjectRecognitionActionFeedback() : base()
        {
            this.feedback = new ObjectRecognitionFeedback();
        }

        public ObjectRecognitionActionFeedback(HeaderMsg header, GoalStatusMsg status, ObjectRecognitionFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static ObjectRecognitionActionFeedback Deserialize(MessageDeserializer deserializer) => new ObjectRecognitionActionFeedback(deserializer);

        ObjectRecognitionActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = ObjectRecognitionFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
