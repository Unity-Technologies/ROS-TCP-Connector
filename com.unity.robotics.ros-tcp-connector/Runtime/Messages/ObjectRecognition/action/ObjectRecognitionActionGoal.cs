using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.ObjectRecognition
{
    public class ObjectRecognitionActionGoal : ActionGoal<ObjectRecognitionGoal>
    {
        public const string k_RosMessageName = "object_recognition_msgs-master/ObjectRecognitionActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public ObjectRecognitionActionGoal() : base()
        {
            this.goal = new ObjectRecognitionGoal();
        }

        public ObjectRecognitionActionGoal(HeaderMsg header, GoalIDMsg goal_id, ObjectRecognitionGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static ObjectRecognitionActionGoal Deserialize(MessageDeserializer deserializer) => new ObjectRecognitionActionGoal(deserializer);

        ObjectRecognitionActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = ObjectRecognitionGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
