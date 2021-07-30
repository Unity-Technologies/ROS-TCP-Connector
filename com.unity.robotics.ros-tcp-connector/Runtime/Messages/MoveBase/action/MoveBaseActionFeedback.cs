using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.MoveBase
{
    public class MoveBaseActionFeedback : ActionFeedback<MoveBaseFeedback>
    {
        public const string k_RosMessageName = "move_base_msgs/MoveBaseActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public MoveBaseActionFeedback() : base()
        {
            this.feedback = new MoveBaseFeedback();
        }

        public MoveBaseActionFeedback(HeaderMsg header, GoalStatusMsg status, MoveBaseFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static MoveBaseActionFeedback Deserialize(MessageDeserializer deserializer) => new MoveBaseActionFeedback(deserializer);

        MoveBaseActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = MoveBaseFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
