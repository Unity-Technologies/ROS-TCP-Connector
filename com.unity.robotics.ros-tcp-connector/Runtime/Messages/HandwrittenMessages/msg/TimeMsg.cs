using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BuiltinInterfaces
{
    public class TimeMsg : Message
    {
#if !ROS2
        public const string k_RosMessageName = "std_msgs/Time";
        public override string RosMessageName => k_RosMessageName;

        public uint sec;
        public uint nanosec;

        public TimeMsg()
        {
            this.sec = 0;
            this.nanosec = 0;
        }

        public TimeMsg(uint sec, uint nanosec)
        {
            this.sec = sec;
            this.nanosec = nanosec;
        }

#else
        public const string k_RosMessageName = "builtin_interfaces/Time";
        public override string RosMessageName => k_RosMessageName;

        //  This message communicates ROS Time defined here:
        //  https://design.ros2.org/articles/clock_and_time.html
        //  The seconds component, valid over all int32 values.
        public int sec;
        //  The nanoseconds component, valid in the range [0, 10e9).
        public uint nanosec;

        public TimeMsg()
        {
            this.sec = 0;
            this.nanosec = 0;
        }

        public TimeMsg(int sec, uint nanosec)
        {
            this.sec = sec;
            this.nanosec = nanosec;
        }
#endif

        public static TimeMsg Deserialize(MessageDeserializer deserializer) => new TimeMsg(deserializer);

        TimeMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.sec);
            deserializer.Read(out this.nanosec);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.sec);
            serializer.Write(this.nanosec);
        }

        public override string ToString()
        {
            return "Time: " +
            "\nsec: " + sec.ToString() +
            "\nnanosec: " + nanosec.ToString();
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
