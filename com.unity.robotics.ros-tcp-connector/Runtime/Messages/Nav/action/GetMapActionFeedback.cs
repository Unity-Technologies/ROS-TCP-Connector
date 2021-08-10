using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.Nav
{
    public class GetMapActionFeedback : ActionFeedback<GetMapFeedback>
    {
        public const string k_RosMessageName = "nav_msgs/GetMapActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public GetMapActionFeedback() : base()
        {
            this.feedback = new GetMapFeedback();
        }

        public GetMapActionFeedback(HeaderMsg header, GoalStatusMsg status, GetMapFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static GetMapActionFeedback Deserialize(MessageDeserializer deserializer) => new GetMapActionFeedback(deserializer);

        GetMapActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = GetMapFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }

    }
}
