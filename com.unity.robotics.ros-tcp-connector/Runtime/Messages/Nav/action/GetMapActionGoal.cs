using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Nav
{
    public class GetMapActionGoal : ActionGoal<GetMapGoal>
    {
        public const string k_RosMessageName = "nav_msgs/GetMapActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public GetMapActionGoal() : base()
        {
            this.goal = new GetMapGoal();
        }

        public GetMapActionGoal(HeaderMsg header, GoalIDMsg goal_id, GetMapGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static GetMapActionGoal Deserialize(MessageDeserializer deserializer) => new GetMapActionGoal(deserializer);

        GetMapActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = GetMapGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
        }

    }
}
