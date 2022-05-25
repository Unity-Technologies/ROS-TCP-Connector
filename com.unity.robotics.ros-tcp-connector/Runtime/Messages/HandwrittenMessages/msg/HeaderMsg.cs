using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Std
{
    public class HeaderMsg : Message
    {
        public const string k_RosMessageName = "std_msgs/Header";
        public override string RosMessageName => k_RosMessageName;


        //  Standard metadata for higher-level stamped data types.
        //  This is generally used to communicate timestamped data
        //  in a particular coordinate frame.

        //  sequence ID: consecutively increasing ID
        // we don't support this field. (it's deprecated in ROS1 and doesn't exist in ROS2). 
        //public uint seq;

        //  Two-integer timestamp that is expressed as seconds and nanoseconds.
        public BuiltinInterfaces.TimeMsg stamp;
        //  Transform frame with which this data is associated.
        public string frame_id;

        public HeaderMsg()
        {
            this.stamp = new BuiltinInterfaces.TimeMsg();
            this.frame_id = "";
        }

        // Note, seq is ignored; provided for backwards compatibility
        public HeaderMsg(uint seq, BuiltinInterfaces.TimeMsg stamp, string frame_id)
        {
            this.stamp = stamp;
            this.frame_id = frame_id;
        }

        public HeaderMsg(BuiltinInterfaces.TimeMsg stamp, string frame_id)
        {
            this.stamp = stamp;
            this.frame_id = frame_id;
        }

        public static HeaderMsg Deserialize(MessageDeserializer deserializer) => new HeaderMsg(deserializer);

        HeaderMsg(MessageDeserializer deserializer)
        {
            uint seq_unused;
            if (!deserializer.IsRos2)
                deserializer.Read(out seq_unused);

            this.stamp = BuiltinInterfaces.TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.frame_id);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            if (!serializer.IsRos2)
                serializer.Write(0U);

            serializer.Write(stamp);
            serializer.Write(this.frame_id);
        }

        public override string ToString()
        {
            return "Header: " +
            "\nstamp: " + stamp.ToString() +
            "\nframe_id: " + frame_id.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
