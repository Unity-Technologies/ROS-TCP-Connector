using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Tf2
{
    public class LookupTransformActionGoal : ActionGoal<LookupTransformGoal>
    {
        public const string k_RosMessageName = "tf2_msgs/LookupTransformActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public LookupTransformActionGoal() : base()
        {
            this.goal = new LookupTransformGoal();
        }

        public LookupTransformActionGoal(HeaderMsg header, GoalIDMsg goal_id, LookupTransformGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static LookupTransformActionGoal Deserialize(MessageDeserializer deserializer) => new LookupTransformActionGoal(deserializer);

        LookupTransformActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = LookupTransformGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
