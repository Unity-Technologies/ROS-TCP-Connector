using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    // Convenience functions for built-in message types
    public static class MessageExtensions
    {
        public static string ToTimestampString(this MTime message)
        {
            // G: format using short date and long time
            return message.ToDateTime().ToString("G") + $"(+{message.nsecs})";
        }

        public static DateTime ToDateTime(this MTime message)
        {
            DateTime time = new DateTime(message.secs);
            time = time.AddMilliseconds(message.nsecs / 1E6);
            return time;
        }

        public static MTime ToMTime(this DateTime dateTime, uint nsecs = 0)
        {
            return new MTime { secs = (uint)dateTime.Ticks, nsecs = (uint)(dateTime.Millisecond * 1E6) };
        }

        public static Color ToUnityColor(this MColorRGBA message)
        {
            return new Color(message.r, message.g, message.b, message.a);
        }

        public static MColorRGBA ToMColorRGBA(this Color color)
        {
            return new MColorRGBA(color.r, color.g, color.b, color.a);
        }
    }
}