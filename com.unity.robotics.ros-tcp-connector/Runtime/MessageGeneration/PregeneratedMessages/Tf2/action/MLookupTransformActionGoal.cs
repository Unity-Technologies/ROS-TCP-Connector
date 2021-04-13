using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Tf2
{
    public class MLookupTransformActionGoal : ActionGoal<MLookupTransformGoal>
    {
        public const string k_RosMessageName = "tf2_msgs/LookupTransformActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public MLookupTransformActionGoal() : base()
        {
            this.goal = new MLookupTransformGoal();
        }

        public MLookupTransformActionGoal(MHeader header, MGoalID goal_id, MLookupTransformGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(this.header.SerializationStatements());
            listOfSerializations.AddRange(this.goal_id.SerializationStatements());
            listOfSerializations.AddRange(this.goal.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.goal_id.Deserialize(data, offset);
            offset = this.goal.Deserialize(data, offset);

            return offset;
        }

    }
}
