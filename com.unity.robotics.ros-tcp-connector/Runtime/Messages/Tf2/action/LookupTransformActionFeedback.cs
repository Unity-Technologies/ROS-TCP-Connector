using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Tf2
{
    public class LookupTransformActionFeedback : ActionFeedback<LookupTransformFeedback>
    {
        public const string k_RosMessageName = "tf2_msgs/LookupTransformActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public LookupTransformActionFeedback() : base()
        {
            this.feedback = new LookupTransformFeedback();
        }

        public LookupTransformActionFeedback(HeaderMsg header, GoalStatusMsg status, LookupTransformFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static LookupTransformActionFeedback Deserialize(MessageDeserializer deserializer) => new LookupTransformActionFeedback(deserializer);

        LookupTransformActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = LookupTransformFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
