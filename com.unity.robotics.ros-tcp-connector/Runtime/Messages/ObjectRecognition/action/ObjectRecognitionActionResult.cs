using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.ObjectRecognition
{
    public class ObjectRecognitionActionResult : ActionResult<ObjectRecognitionResult>
    {
        public const string k_RosMessageName = "object_recognition_msgs-master/ObjectRecognitionActionResult";
        public override string RosMessageName => k_RosMessageName;


        public ObjectRecognitionActionResult() : base()
        {
            this.result = new ObjectRecognitionResult();
        }

        public ObjectRecognitionActionResult(HeaderMsg header, GoalStatusMsg status, ObjectRecognitionResult result) : base(header, status)
        {
            this.result = result;
        }
        public static ObjectRecognitionActionResult Deserialize(MessageDeserializer deserializer) => new ObjectRecognitionActionResult(deserializer);

        ObjectRecognitionActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = ObjectRecognitionResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }

    }
}
