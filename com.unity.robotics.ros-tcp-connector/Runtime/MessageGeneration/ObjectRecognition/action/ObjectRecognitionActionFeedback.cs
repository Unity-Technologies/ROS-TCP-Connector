using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.ObjectRecognition
{
    public class ObjectRecognitionActionFeedback : ActionFeedback<ObjectRecognitionFeedback>
    {
        public const string RosMessageName = "object_recognition_msgs-master/ObjectRecognitionActionFeedback";

        public ObjectRecognitionActionFeedback() : base()
        {
            this.feedback = new ObjectRecognitionFeedback();
        }

        public ObjectRecognitionActionFeedback(Header header, GoalStatus status, ObjectRecognitionFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(this.header.SerializationStatements());
            listOfSerializations.AddRange(this.status.SerializationStatements());
            listOfSerializations.AddRange(this.feedback.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.status.Deserialize(data, offset);
            offset = this.feedback.Deserialize(data, offset);

            return offset;
        }

    }
}
