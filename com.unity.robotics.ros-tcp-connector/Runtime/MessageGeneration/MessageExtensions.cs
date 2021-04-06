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
        public static byte[] EncodingConversion(byte[] toConvert, string from, int width, int height)
        {
            if (from == "mono8" || from == "mono16" || from == "rgb8" || from == "rgba8") 
            {
                Debug.Log($"Conversion not supported from {from}, returning original byte array!");
                return toConvert;
            }

            Dictionary<int, int> channelConversion = new Dictionary<int, int>();

            // TODO: Is it safe to assume 8UC3 will default to bgr8?
            channelConversion.Add(0, 2); // B -> R 
            channelConversion.Add(1, 1); // G -> G
            channelConversion.Add(2, 0); // R -> B
            channelConversion.Add(3, 3); // A -> A
            
            int idx = 0;
            int pixel = 0;
            int flipIdx;
            int tmpR;
            int tmpC;
            int tmpH = height - 1;
            int tmpW = width * 3;

            byte[] converted = new byte[toConvert.Length];

            // Bit shift BGR->RGB and flip across X axis
            for (int i = 0; i < toConvert.Length; i++)
            {
                pixel = i / 3;
                tmpR = tmpH - (i / tmpW);
                tmpC = i % tmpW;
                flipIdx = ((tmpR * tmpW) + tmpC);
                converted[flipIdx] = toConvert[pixel * 3 + channelConversion[idx]];

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
                    return TextureFormat.RGB24;
                case "rgb8":
                    return TextureFormat.RGB24;
                case "bgra8":
                    return TextureFormat.RGBA32;
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
            var data = EncodingConversion(message.data, message.encoding, (int)message.width, (int)message.height);
            tex.LoadRawTextureData(data);
            tex.Apply();
            return tex;
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

        public enum JoystickRegion 
        {
            BSouth = 0,
            BEast = 1,
            BWest = 2,
            BNorth = 3,
            LB = 4,
            RB = 5,
            Back = 6,
            Start = 7,
            Power = 8,
            LPress = 9,
            RPress = 10,
            LStick,
            RStick,
            LT,
            RT,
            DPad
        };

        public static Texture2D TextureFromJoy(this MJoy message, JoystickRegion region, int layout=0) 
        {
            Color[] colorHighlight = new Color[100];
            for (int i = 0; i < colorHighlight.Length; i++)
            {
                colorHighlight[i] = Color.red;
            }

            Color[] colorPress = new Color[2500];
            for (int i = 0; i < colorPress.Length; i++)
            {
                colorPress[i] = Color.blue;
            }

            Texture2D tex;
            int x = 0;
            int y = 0;
            int lAxisX = 0;
            int lAxisY = 1;
            int rAxisX = 2;
            int rAxisY = 3;
            int ltAxis = 4;
            int rtAxis = 5;
            int dAxisX = 6;
            int dAxisY = 7;
            int buttonIdx = (int)region;

            if (layout == 0) 
            {
                // Dualshock 4
                switch (region)
                {
                    case JoystickRegion.BSouth:
                        buttonIdx = 0;
                        break;
                    case JoystickRegion.BEast:
                        buttonIdx = 1;
                        break;
                    case JoystickRegion.BWest:
                        buttonIdx = 3;
                        break;
                    case JoystickRegion.BNorth:
                        buttonIdx = 2;
                        break;
                    case JoystickRegion.LT:
                        ltAxis = 2;
                        break;
                    case JoystickRegion.Back:
                        buttonIdx = 8;
                        break;
                    case JoystickRegion.Start:
                        buttonIdx = 9;
                        break;
                    case JoystickRegion.Power:
                        buttonIdx = 10;
                        break;
                    case JoystickRegion.LStick:
                        buttonIdx = 11;
                        break;
                    case JoystickRegion.RStick:
                        rAxisX = 3;
                        rAxisY = 4;
                        buttonIdx = 12;
                        break;
                }
            }
            else if (layout == 1) 
            {
                // Microsoft Xbox 360 Wireless Controller for Windows
                switch (region)
                {
                    case JoystickRegion.Back:
                        buttonIdx = 14;
                        break;
                }
            }
            else if (layout == 2) 
            {
                // Microsoft Xbox 360 Wireless Controller for Linux
            }
            else if (layout == 3) 
            {
                // Microsoft Xbox 360 Wired Controller for Linux
                switch (region)
                {
                    case JoystickRegion.LT:
                        ltAxis = 2;
                        break;
                    case JoystickRegion.RStick:
                        rAxisX = 3;
                        rAxisY = 4;
                        break;
                }
            }
            else if (layout == 4) 
            {
                // Logitech Wireless Gamepad F710
                switch (region)
                {
                    case JoystickRegion.BSouth:
                        buttonIdx = 1;
                        break;
                    case JoystickRegion.BEast:
                        buttonIdx = 2;
                        break;
                    case JoystickRegion.BWest:
                        buttonIdx = 0;
                        break;
                    case JoystickRegion.BNorth:
                        buttonIdx = 3;
                        break;
                    case JoystickRegion.LT:
                        buttonIdx = 6;
                        break;
                    case JoystickRegion.RT:
                        buttonIdx = 7;
                        break;
                    case JoystickRegion.Back:
                        buttonIdx = 8;
                        break;
                    case JoystickRegion.Start:
                        buttonIdx = 9;
                        break;
                    case JoystickRegion.LPress:
                        buttonIdx = 10;
                        break;
                    case JoystickRegion.RPress:
                        buttonIdx = 11;
                        break;
                }
            }

            if ((int)region <= 10) 
            {
                // Define small button context
                tex = new Texture2D(10, 10);
                if (message.buttons[buttonIdx] == 1)
                    tex.SetPixels(0, 0, 10, 10, colorHighlight);
            }
            else {
                // Axes
                switch (region)
                {
                    case JoystickRegion.LStick:
                        tex = new Texture2D(50, 50);
                        x = (Mathf.FloorToInt(message.axes[lAxisX] * -20) + tex.width / 2);
                        y = (Mathf.FloorToInt(message.axes[lAxisY] * 20) + tex.height / 2);
                        if (message.buttons[buttonIdx] == 1)
                            tex.SetPixels(0, 0, 50, 50, colorPress);
                        tex.SetPixels(x - 5, y - 5, 10, 10, colorHighlight);
                        break;
                    case JoystickRegion.RStick:
                        tex = new Texture2D(50, 50);
                        x = (Mathf.FloorToInt(message.axes[rAxisX] * -20) + tex.width / 2);
                        y = (Mathf.FloorToInt(message.axes[rAxisY] * 20) + tex.height / 2);
                        if (message.buttons[buttonIdx] == 1)
                            tex.SetPixels(0, 0, 50, 50, colorPress);
                        tex.SetPixels(x - 5, y - 5, 10, 10, colorHighlight);
                        break;
                    case JoystickRegion.DPad:
                        tex = new Texture2D(50, 50);
                        x = (Mathf.FloorToInt(message.axes[dAxisX] * -20) + tex.width / 2);
                        y = (Mathf.FloorToInt(message.axes[dAxisY] * 20) + tex.height / 2);
                        tex.SetPixels(x - 5, y - 5, 10, 10, colorHighlight);
                        break;
                    case JoystickRegion.LT:
                        tex = new Texture2D(25, 50);
                        y = Mathf.FloorToInt(message.axes[ltAxis] * 20) + tex.height / 2;
                        tex.SetPixels(0, y - 2, 25, 4, colorHighlight);
                        break;
                    case JoystickRegion.RT:
                        tex = new Texture2D(25, 50);
                        y = Mathf.FloorToInt(message.axes[rtAxis] * 20) + tex.height / 2;
                        tex.SetPixels(0, y - 2, 25, 4, colorHighlight);
                        break;
                    default:
                        tex = new Texture2D(1,1);
                        break;
                }
            }
            tex.Apply();
            return tex;
        }

        public enum BatteryStateStatusConstants
        {
            POWER_SUPPLY_STATUS_UNKNOWN = 0,
            POWER_SUPPLY_STATUS_CHARGING = 1,
            POWER_SUPPLY_STATUS_DISCHARGING = 2,
            POWER_SUPPLY_STATUS_NOT_CHARGING = 3,
            POWER_SUPPLY_STATUS_FULL = 4
        }

        public enum BatteryStateHealthConstants
        {
            POWER_SUPPLY_HEALTH_UNKNOWN = 0,
            POWER_SUPPLY_HEALTH_GOOD = 1,
            POWER_SUPPLY_HEALTH_OVERHEAT = 2,
            POWER_SUPPLY_HEALTH_DEAD = 3,
            POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4,
            POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5,
            POWER_SUPPLY_HEALTH_COLD = 6,
            POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7,
            POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8
        }
        public enum BatteryStateTechnologyConstants
        {
            POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0,
            POWER_SUPPLY_TECHNOLOGY_NIMH = 1,
            POWER_SUPPLY_TECHNOLOGY_LION = 2,
            POWER_SUPPLY_TECHNOLOGY_LIPO = 3,
            POWER_SUPPLY_TECHNOLOGY_LIFE = 4,
            POWER_SUPPLY_TECHNOLOGY_NICD = 5,
            POWER_SUPPLY_TECHNOLOGY_LIMN = 6
        }
    }
}