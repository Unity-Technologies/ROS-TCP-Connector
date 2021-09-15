using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BuiltinInterfaces
{
    public class DurationMsg : Message
    {
#if !ROS2
        public const string k_RosMessageName = "builtin_interfaces/Duration";
        public override string RosMessageName => k_RosMessageName;

        public int sec;
        public int nanosec;

        public DurationMsg()
        {
            this.sec = 0;
            this.nanosec = 0;
        }

        public DurationMsg(int sec, int nanosec)
        {
            this.sec = sec;
            this.nanosec = nanosec;
        }
#else
        public const string k_RosMessageName = "builtin_interfaces/Duration";
        public override string RosMessageName => k_RosMessageName;


        //  Duration defines a period between two time points.
        //  Messages of this datatype are of ROS Time following this design:
        //  https://design.ros2.org/articles/clock_and_time.html
        //  Seconds component, range is valid over any possible int32 value.
        public int sec;
        //  Nanoseconds component in the range of [0, 10e9).
        public uint nanosec;

        public DurationMsg()
        {
            this.sec = 0;
            this.nanosec = 0;
        }

        public DurationMsg(int sec, uint nanosec)
        {
            this.sec = sec;
            this.nanosec = nanosec;
        }
#endif

        public static DurationMsg Deserialize(MessageDeserializer deserializer) => new DurationMsg(deserializer);

        DurationMsg(MessageDeserializer deserializer)
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
            return "Duration: " +
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
