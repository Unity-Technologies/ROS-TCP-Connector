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

        public uint secs;
        public uint nsecs;

        // for convenience when writing ROS2 agnostic code
        public int sec { get => (int)secs; set => secs = (uint)value; }
        public uint nanosec { get => nsecs; set => nsecs = value; }

        public TimeMsg()
        {
            this.secs = 0;
            this.nsecs = 0;
        }

        public TimeMsg(uint secs, uint nsecs)
        {
            this.secs = secs;
            this.nsecs = nsecs;
        }

        public static TimeMsg Deserialize(MessageDeserializer deserializer) => new TimeMsg(deserializer);

        TimeMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.secs);
            deserializer.Read(out this.nsecs);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.secs);
            serializer.Write(this.nsecs);
        }

        public override string ToString()
        {
            return "Time: " +
            "\nsecs: " + secs.ToString() +
            "\nnsecs: " + nsecs.ToString();
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

        // for convenience when writing ROS2 agnostic code
        public uint secs { get => (uint)sec; set => sec = (int)value; }
        public uint nsecs { get => nanosec; set => nanosec = value; }

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
#endif

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
