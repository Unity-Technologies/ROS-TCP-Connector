using RosMessageTypes.Sensor;
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

        /// <summary>
        /// Converts a byte array from BGR to RGB.
        /// </summary>
        public static byte[] EncodingConversion(byte[] toConvert, string from)
        {
            Dictionary<int, int> channelConversion = new Dictionary<int, int>();

            // TODO: Is it safe to assume 8UC3 will default to bgr8?

            // Three channels
            if (from[from.Length - 1] == 3) 
            {
                channelConversion.Add(0, 2); // B -> R
                channelConversion.Add(1, 1); // G -> G
                channelConversion.Add(2, 0); // R -> B
            }
            else {
                channelConversion.Add(0, 2); // B -> R 
                channelConversion.Add(1, 1); // G -> G
                channelConversion.Add(2, 0); // R -> B
                channelConversion.Add(3, 3); // A -> A
            }
            
            int idx = 0;
            int pixel = 0;

            byte[] converted = new byte[toConvert.Length];

            for (int i = 0; i < toConvert.Length; i++)
            {
                pixel = i / 3;
                converted[i] = toConvert[pixel * 3 + channelConversion[idx]];

                idx = (idx + 1) % 3;
            }

            return converted;
        }

        public static TextureFormat EncodingToTextureFormat(string encoding)
        {
            switch (encoding)
            {
                case "8UC1":
                    return TextureFormat.R8;
                case "8UC2":
                    return TextureFormat.RG16;
                case "8UC3":
                    // TODO: arrives BGR, needs RGB conversion
                    return TextureFormat.RGB24;
                case "8UC4":
                    return TextureFormat.RGBA32;
                case "8SC1":
                    return TextureFormat.R8;
                case "8SC2":
                    return TextureFormat.RG16;
                case "8SC3":
                    return TextureFormat.RGB24;
                case "8SC4":
                    return TextureFormat.RGBA32;
                case "16UC1":
                    return TextureFormat.R16;
                case "16UC2":
                    return TextureFormat.RG32;
                case "16UC3":
                    return TextureFormat.RGB48;
                case "16UC4":
                    return TextureFormat.RGBA64;
                case "16SC1":
                    return TextureFormat.R16;
                case "16SC2":
                    return TextureFormat.RG32;
                case "16SC3":
                    return TextureFormat.RGB48;
                case "16SC4":
                    return TextureFormat.RGBA64;
                case "32SC1":
                    throw new NotImplementedException();
                case "32SC2":
                    throw new NotImplementedException();
                case "32SC3":
                    throw new NotImplementedException();
                case "32SC4":
                    throw new NotImplementedException();
                case "32FC1":
                    return TextureFormat.RFloat;
                case "32FC2":
                    return TextureFormat.RGFloat;
                case "32FC3":
                    throw new NotImplementedException();
                case "32FC4":
                    return TextureFormat.RGBAFloat;
                case "64FC1":
                    return TextureFormat.RGB24;
                case "64FC2":
                    return TextureFormat.RGB24;
                case "64FC3":
                    return TextureFormat.RGB24;
                case "64FC4":
                    return TextureFormat.RGB24;
                case "mono8":
                    return TextureFormat.R8;
                case "mono16":
                    return TextureFormat.R16;
                case "bgr8":
                    // return TextureFormat.RGB24;
                    throw new NotImplementedException();
                case "rgb8":
                    return TextureFormat.RGB24;
                case "bgra8":
                    throw new NotImplementedException();
                case "rgba8":
                    return TextureFormat.RGBA32;
            }
            return TextureFormat.RGB24;
        }

        public static Texture2D ToTexture2D(this MCompressedImage message)
        {
            var tex = new Texture2D(1, 1);
            tex.LoadImage(message.data);
            return tex;
        }

        public static Texture2D ToTexture2D(this MImage message)
        {
            var tex = new Texture2D((int)message.width, (int)message.height, EncodingToTextureFormat(message.encoding), false);
            var data = EncodingConversion(message.data, message.encoding);
            tex.LoadRawTextureData(message.data);
            tex.Apply();
            return tex;

            // TODO: Needs to flip image 180º
            // Texture2D modified = new Texture2D(tex.width, tex.height);
            // for (int i = 0; i < tex.width; i++)
            // {
            //     for (int j = 0; j < tex.height; j++)
            //     {
            //         var color = tex.GetPixel(i, tex.height - j);
            //         modified.SetPixel(i, j, color);
            //     }
            // }
            // modified.Apply();
            // return modified;
        }

        public static MCompressedImage ToMCompressedImage(this Texture2D tex, string format="jpeg")
        {
            var data = tex.GetRawTextureData();
            return new MCompressedImage(new MHeader(), format, data);
        }

        public static MImage ToMImage(this Texture2D tex, string encoding="RGBA", byte isBigEndian=0, uint step=4)
        {
            var data = tex.GetRawTextureData();
            return new MImage(new MHeader(), (uint)tex.width, (uint)tex.height, encoding, isBigEndian, step, data);
        }
    }
}