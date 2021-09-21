using System;
using RosMessageTypes.BuiltinInterfaces;
using UnityEngine;

namespace Unity.Robotics.Core
{
    public readonly struct TimeStamp
    {
        public const double k_NanosecondsInSecond = 1e9f;

        // TODO: specify base time this stamp is measured against (Sim 0, time since application start, etc.)
        public readonly int Seconds;
        public readonly uint NanoSeconds;

        // (From Unity Time.time)
        public TimeStamp(double timeInSeconds)
        {
            var sec = Math.Floor(timeInSeconds);
            var nsec = (timeInSeconds - sec) * k_NanosecondsInSecond;

            // TODO: Check for negatives to ensure safe cast
            Seconds = (int)sec;
            NanoSeconds = (uint)nsec;
        }

        // (From a ROS2 Time message)
        TimeStamp(int sec, uint nsec)
        {
            Seconds = sec;
            NanoSeconds = nsec;
        }

        // NOTE: We could define these operators in a transport-specific extension package
        public static implicit operator TimeMsg(TimeStamp stamp)
        {
            return new TimeMsg(stamp.Seconds, stamp.NanoSeconds);
        }

        public static implicit operator TimeStamp(TimeMsg stamp)
        {
            return new TimeStamp(stamp.sec, stamp.nanosec);
        }
    }
}
