using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.MoveBase
{
    public class MoveBaseActionGoal : ActionGoal<MoveBaseGoal>
    {
        public const string k_RosMessageName = "move_base_msgs/MoveBaseActionGoal";


        public MoveBaseActionGoal() : base()
        {
            this.goal = new MoveBaseGoal();
        }

        public MoveBaseActionGoal(HeaderMsg header, GoalIDMsg goal_id, MoveBaseGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static MoveBaseActionGoal Deserialize(MessageDeserializer deserializer) => new MoveBaseActionGoal(deserializer);

        MoveBaseActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = MoveBaseGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
