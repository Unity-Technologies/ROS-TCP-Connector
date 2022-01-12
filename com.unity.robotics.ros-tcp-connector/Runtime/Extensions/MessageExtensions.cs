using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public enum BatteryState_Status_Constants
    {
        UNKNOWN = 0,
        CHARGING = 1,
        DISCHARGING = 2,
        NOT_CHARGING = 3,
        FULL = 4
    };

    public enum BatteryState_Health_Constants
    {
        UNKNOWN = 0,
        GOOD = 1,
        OVERHEAT = 2,
        DEAD = 3,
        OVERVOLTAGE = 4,
        UNSPEC_FAILURE = 5,
        COLD = 6,
        WATCHDOG_TIMER_EXPIRE = 7,
        SAFETY_TIMER_EXPIRE = 8
    };

    public enum BatteryState_Technology_Constants
    {
        UNKNOWN = 0,
        NIMH = 1,
        LION = 2,
        LIPO = 3,
        LIFE = 4,
        NICD = 5,
        LIMN = 6
    };

    public enum JoyFeedback_Type_Constants
    {
        LED = 0,
        RUMBLE = 1,
        BUZZER = 2,
    };

    public enum JoyRegion
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
        LStick, RStick, LT, RT, DPad, lAxisX, lAxisY,
        rAxisX, rAxisY, ltAxis, rtAxis, dAxisX, dAxisY
    };

    public enum NavSatStatus_Type_Constants
    {
        NO_FIX = -1,        // unable to fix position
        FIX = 0,        // unaugmented fix
        SBAS_FIX = 1,        // with satellite-based augmentation
        GBAS_FIX = 2         // with ground-based augmentation
    };

    public enum NavSatStatus_Service_Constants
    {
        GPS = 1,
        GLONASS = 2,
        COMPASS = 4,      // includes BeiDou.
        GALILEO = 8
    };

    public enum NavSatFix_Covariance_Constants
    {
        UNKNOWN = 0,
        APPROXIMATED = 1,
        DIAGONAL_KNOWN = 2,
        KNOWN = 3
    };

    public enum Range_RadiationType_Constants
    {
        ULTRASOUND = 0,
        INFRARED = 1
    };

    public enum PointField_Format_Constants
    {
        INT8 = 1,
        UINT8 = 2,
        INT16 = 3,
        UINT16 = 4,
        INT32 = 5,
        UINT32 = 6,
        FLOAT32 = 7,
        FLOAT64 = 8
    }

    // Convenience functions for built-in message types
    public static class MessageExtensions
    {
        public static string ToTimestampString(this TimeMsg message)
        {
            // G: format using short date and long time
            return message.ToDateTime().ToString("G") + $"(+{message.nanosec})";
        }

        public static long ToLongTime(this TimeMsg message)
        {
            return (long)message.sec << 32 | message.nanosec;
        }

        public static DateTime ToDateTime(this TimeMsg message)
        {
            DateTime time = new DateTime(message.sec);
            time = time.AddMilliseconds(message.nanosec / 1E6);
            return time;
        }

        // public static TimeMsg TimeMsg(this DateTime dateTime)
        // {
        //     return new TimeMsg { sec = Convert.ToUInt32(dateTime.Ticks), nanosec = Convert.ToUInt32(dateTime.Millisecond * 1E6) };
        // }

        public static Color ToUnityColor(this ColorRGBAMsg message)
        {
            return new Color(message.r, message.g, message.b, message.a);
        }

        public static ColorRGBAMsg ColorRGBAMsg(this Color color)
        {
            return new ColorRGBAMsg(color.r, color.g, color.b, color.a);
        }

        /// <summary>
        /// Converts a byte array from BGR to RGB.
        /// </summary>
        static byte[] EncodingConversion(ImageMsg image, bool convertBGR = true, bool flipY = true)
        {
            // Number of channels in this encoding
            int channels = image.GetNumChannels();

            if (!image.EncodingRequiresBGRConversion())
                convertBGR = false;

            // If no modifications are necessary, return original array
            if (!convertBGR && !flipY)
                return image.data;

            int channelStride = image.GetBytesPerChannel();
            int pixelStride = channelStride * channels;
            int rowStride = pixelStride * (int)image.width;

            if (flipY)
            {
                ReverseInBlocks(image.data, rowStride, (int)image.height);
            }

            if (convertBGR)
            {
                // given two channels, we swap R with G (distance = 1).
                // given three or more channels, we swap R with B (distance = 2).
                int swapDistance = channels == 2 ? channelStride : channelStride * 2;
                int dataLength = (int)image.width * (int)image.height * pixelStride;

                if (channelStride == 1)
                {
                    // special case for the 1-byte-per-channel formats: avoid the inner loop
                    for (int pixelIndex = 0; pixelIndex < dataLength; pixelIndex += pixelStride)
                    {
                        int swapB = pixelIndex + swapDistance;
                        byte temp = image.data[pixelIndex];
                        image.data[pixelIndex] = image.data[swapB];
                        image.data[swapB] = temp;
                    }
                }
                else
                {
                    for (int pixelIndex = 0; pixelIndex < dataLength; pixelIndex += pixelStride)
                    {
                        int channelEndByte = pixelIndex + channelStride;
                        for (int byteIndex = pixelIndex; byteIndex < channelEndByte; byteIndex++)
                        {
                            int swapB = byteIndex + swapDistance;
                            byte temp = image.data[byteIndex];
                            image.data[byteIndex] = image.data[swapB];
                            image.data[swapB] = temp;
                        }
                    }
                }
            }
            return image.data;
        }

        static byte[] s_ScratchSpace;

        static void ReverseInBlocks(byte[] array, int blockSize, int numBlocks)
        {
            if (blockSize * numBlocks > array.Length)
            {
                Debug.LogError($"Invalid ReverseInBlocks, array length is {array.Length}, should be at least {blockSize * numBlocks}");
                return;
            }

            if (s_ScratchSpace == null || s_ScratchSpace.Length < blockSize)
                s_ScratchSpace = new byte[blockSize];

            int startBlockIndex = 0;
            int endBlockIndex = ((int)numBlocks - 1) * blockSize;

            while (startBlockIndex < endBlockIndex)
            {
                Buffer.BlockCopy(array, startBlockIndex, s_ScratchSpace, 0, blockSize);
                Buffer.BlockCopy(array, endBlockIndex, array, startBlockIndex, blockSize);
                Buffer.BlockCopy(s_ScratchSpace, 0, array, endBlockIndex, blockSize);
                startBlockIndex += blockSize;
                endBlockIndex -= blockSize;
            }
        }

        static void ReverseInBlocks<T1, T2>(T1[] fromArray, T2[] toArray, int blockSize, int numBlocks)
        {
            int startBlockIndex = 0;
            int endBlockIndex = ((int)numBlocks - 1) * blockSize;

            while (startBlockIndex < endBlockIndex)
            {
                Buffer.BlockCopy(fromArray, startBlockIndex, toArray, endBlockIndex, blockSize);
                Buffer.BlockCopy(fromArray, endBlockIndex, toArray, startBlockIndex, blockSize);
                startBlockIndex += blockSize;
                endBlockIndex -= blockSize;
            }
        }

        public static void DebayerConvert(this ImageMsg image, bool flipY = true)
        {
            int channelStride = image.GetBytesPerChannel();
            int width = (int)image.width;
            int height = (int)image.height;
            int rowStride = width * channelStride;
            int dataSize = rowStride * height;
            int finalPixelStride = channelStride * 4;

            int[] reorderIndices;
            switch (image.encoding)
            {
                case "bayer_rggb8":
                    reorderIndices = new int[] { 0, 1, width + 1 };
                    break;
                case "bayer_bggr8":
                    reorderIndices = new int[] { width + 1, 1, 0 };
                    break;
                case "bayer_gbrg8":
                    reorderIndices = new int[] { width, 0, 1 };
                    break;
                case "bayer_grbg8":
                    reorderIndices = new int[] { 1, 0, width };
                    break;
                case "bayer_rggb16":
                    reorderIndices = new int[] { 0, 1, 2, 3, rowStride + 2, rowStride + 3 };
                    break;
                case "bayer_bggr16":
                    reorderIndices = new int[] { rowStride + 2, rowStride + 3, 2, 3, 0, 1 };
                    break;
                case "bayer_gbrg16":
                    reorderIndices = new int[] { rowStride, rowStride + 1, 0, 1, 2, 3 };
                    break;
                case "bayer_grbg16":
                    reorderIndices = new int[] { 2, 3, 0, 1, rowStride, rowStride + 1 };
                    break;
                default:
                    return;
            }

            if (flipY)
            {
                ReverseInBlocks(image.data, rowStride * 2, (int)image.height / 2);
            }

            if (s_ScratchSpace == null || s_ScratchSpace.Length < rowStride * 2)
                s_ScratchSpace = new byte[rowStride * 2];

            int rowStartIndex = 0;
            while (rowStartIndex < dataSize)
            {
                Buffer.BlockCopy(image.data, rowStartIndex, s_ScratchSpace, 0, rowStride * 2);
                int pixelReadIndex = 0;
                int pixelWriteIndex = rowStartIndex;
                while (pixelReadIndex < rowStride)
                {
                    for (int Idx = 0; Idx < reorderIndices.Length; ++Idx)
                    {
                        image.data[pixelWriteIndex + Idx] = s_ScratchSpace[pixelReadIndex + reorderIndices[Idx]];
                    }
                    image.data[pixelWriteIndex + reorderIndices.Length] = 255;
                    if (channelStride == 2)
                        image.data[pixelWriteIndex + reorderIndices.Length + 1] = 255;
                    pixelReadIndex += channelStride * 2;
                    pixelWriteIndex += finalPixelStride;
                }
                rowStartIndex += rowStride * 2;
            }

            image.width = image.width / 2;
            image.height = image.height / 2;
            image.encoding = channelStride == 1 ? "rgba8" : "rgba16";
            image.step = (uint)(channelStride * image.width);
        }

        public static int GetNumChannels(this ImageMsg image)
        {
            switch (image.encoding)
            {
                case "8SC1":
                case "8UC1":
                case "16SC1":
                case "16UC1":
                case "32FC1":
                case "32SC1":
                case "64FC1":
                case "mono8":
                case "mono16":
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                case "bayer_rggb16":
                case "bayer_bggr16":
                case "bayer_gbrg16":
                case "bayer_grbg16":
                    return 1;
                case "8SC2":
                case "8UC2":
                case "16SC2":
                case "16UC2":
                case "32FC2":
                case "32SC2":
                case "64FC2":
                    return 2;
                case "8SC3":
                case "8UC3":
                case "16SC3":
                case "16UC3":
                case "32FC3":
                case "32SC3":
                case "64FC3":
                case "bgr8":
                case "rgb8":
                    return 3;
                case "8SC4":
                case "8UC4":
                case "16SC4":
                case "16UC4":
                case "32FC4":
                case "32SC4":
                case "64FC4":
                case "bgra8":
                case "rgba8":
                    return 4;
            }
            return 4;
        }

        public static bool IsBayerEncoded(this ImageMsg image)
        {
            switch (image.encoding)
            {
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                case "bayer_rggb16":
                case "bayer_bggr16":
                case "bayer_gbrg16":
                case "bayer_grbg16":
                    return true;
                default:
                    return false;
            }
        }

        public static bool EncodingRequiresBGRConversion(this ImageMsg image)
        {
            switch (image.encoding)
            {
                case "8SC1":
                case "8UC1":
                case "16SC1":
                case "16UC1":
                case "32FC1":
                case "32SC1":
                case "64FC1":
                case "mono8":
                case "mono16":
                    // single channel = nothing to swap
                    return false;
                case "8UC4":
                case "8SC4":
                case "bgra8":
                    return false; // raw BGRA32 texture format
                case "rgb8":
                    return false; // raw RGB24 texture format
                case "rgba8":
                    return false; // raw RGB32 texture format
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                    return false; // bayer has its own conversions needed
                default:
                    return true;
            }
        }

        public static int GetBytesPerChannel(this ImageMsg image)
        {
            switch (image.encoding)
            {
                case "8SC1":
                case "8SC2":
                case "8SC3":
                case "8SC4":
                case "8UC1":
                case "8UC2":
                case "8UC3":
                case "8UC4":
                case "mono8":
                case "bgr8":
                case "rgb8":
                case "bgra8":
                case "rgba8":
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                    return 1;
                case "16SC1":
                case "16SC2":
                case "16SC3":
                case "16SC4":
                case "16UC1":
                case "16UC2":
                case "16UC3":
                case "16UC4":
                case "mono16":
                case "bayer_rggb16":
                case "bayer_bggr16":
                case "bayer_gbrg16":
                case "bayer_grbg16":
                    return 2;
                case "32FC1":
                case "32SC1":
                case "32FC2":
                case "32SC2":
                case "32FC3":
                case "32SC3":
                case "32FC4":
                case "32SC4":
                    return 4;
                case "64FC1":
                case "64FC2":
                case "64FC3":
                case "64FC4":
                    return 8;
            }
            return 1;
        }

        public static TextureFormat GetTextureFormat(this ImageMsg image)
        {
            switch (image.encoding)
            {
                case "8UC1":
                case "8SC1":
                    return TextureFormat.R8;
                case "8UC2":
                case "8SC2":
                    return TextureFormat.RG16;
                case "8UC3":
                case "8SC3":
                    return TextureFormat.RGB24;
                case "8UC4":
                case "8SC4":
                case "bgra8":
                    return TextureFormat.BGRA32; // unity supports these natively
                case "16UC1":
                case "16SC1":
                    return TextureFormat.R16;
                case "16UC2":
                case "16SC2":
                    return TextureFormat.RG32;
                case "16UC3":
                case "16SC3":
                    return TextureFormat.RGB48;
                case "16UC4":
                case "16SC4":
                    return TextureFormat.RGBA64;
                case "32SC1":
                case "32SC2":
                case "32SC3":
                case "32SC4":
                    throw new NotImplementedException("32 bit integer texture formats are not supported");
                case "32FC1":
                    return TextureFormat.RFloat;
                case "32FC2":
                    return TextureFormat.RGFloat;
                case "32FC3":
                    throw new NotImplementedException("32FC3 texture format is not supported");
                case "32FC4":
                    return TextureFormat.RGBAFloat;
                case "64FC1":
                case "64FC2":
                case "64FC3":
                case "64FC4":
                    throw new NotImplementedException("Double precision texture formats are not supported");
                case "mono8":
                    return TextureFormat.R8;
                case "mono16":
                    return TextureFormat.R16;
                case "bgr8":
                    return TextureFormat.RGB24;
                case "rgb8":
                    return TextureFormat.RGB24; // unity supports this natively
                case "rgba8":
                    return TextureFormat.RGBA32; // unity supports this natively
                case "bayer_rggb8":
                case "bayer_bggr8":
                case "bayer_gbrg8":
                case "bayer_grbg8":
                    return TextureFormat.R8;
                case "bayer_rggb16":
                case "bayer_bggr16":
                case "bayer_gbrg16":
                case "bayer_grbg16":
                    return TextureFormat.R16;
            }
            return TextureFormat.RGB24;
        }

        public static Texture2D ToTexture2D(this CompressedImageMsg message)
        {
            var tex = new Texture2D(1, 1);
            tex.LoadImage(message.data);
            return tex;
        }

        public static Texture2D ToTexture2D(this ImageMsg message, bool debayer = false, bool convertBGR = true, bool flipY = true)
        {
            Texture2D tex;
            byte[] data;
            if (debayer && message.IsBayerEncoded())
            {
                tex = new Texture2D((int)message.width / 2, (int)message.height / 2, TextureFormat.RGBA32, false);
                message.DebayerConvert(flipY);
                data = message.data;
            }
            else
            {
                tex = new Texture2D((int)message.width, (int)message.height, message.GetTextureFormat(), false);
                data = EncodingConversion(message, convertBGR, flipY);
            }

            tex.LoadRawTextureData(data);
            tex.Apply();
            return tex;
        }

        /// <summary>
        /// Finds the world coordinates of an image coordinate based on the CameraInfo matrices
        /// K matrix     P matrix
        /// 0 1 2        0 1 2 3
        /// 3 4 5        4 5 6 7
        /// 6 7 8        8 9 10 11
        /// </summary>
        /// <param name="x">x coord to map</param>
        /// <param name="y">y coord to map</param>
        /// <param name="P">Projection matrix (3x4)</param>
        /// <param name="K">Camera intrinsics (3x3)</param>
        /// <returns></returns>
        static PointMsg FindPixelProjection(int x, int y, double[] P, double[][] invK)
        {
            PointMsg worldCoord = new PointMsg(x, y, 1);

            return new PointMsg((float)invK[0][0] * worldCoord.x + (float)invK[0][1] * worldCoord.y + (float)invK[0][2] * worldCoord.z, (float)invK[1][0] * worldCoord.x + (float)invK[1][1] * worldCoord.y + (float)invK[1][2] * worldCoord.z, (float)invK[2][0] * worldCoord.x + (float)invK[2][1] * worldCoord.y + (float)invK[2][2] * worldCoord.z);
        }

        public static PointMsg[] GetPixelsInWorld(this CameraInfoMsg cameraInfo)
        {
            List<PointMsg> res = new List<PointMsg>();

            double[][] formatK = new double[][] {
                new double[] { cameraInfo.k[0], cameraInfo.k[1], cameraInfo.k[2] },
                new double[] { cameraInfo.k[3], cameraInfo.k[4], cameraInfo.k[5] },
                new double[] { cameraInfo.k[6], cameraInfo.k[7], cameraInfo.k[8] },
            };

            var inverseK = MatrixExtensions.MatrixInverse(formatK);

            for (int i = 0; i < cameraInfo.width; i++)
            {
                for (int j = 0; j < cameraInfo.height; j++)
                {
                    res.Add(FindPixelProjection(i, j, cameraInfo.p, inverseK));
                }
            }

            return res.ToArray();
        }

        public static Texture2D ApplyCameraInfoProjection(this Texture2D tex, CameraInfoMsg cameraInfo)
        {
            var newTex = new Texture2D(tex.width, tex.height);

            double[][] formatK = new double[][] {
                new double[] { cameraInfo.k[0], cameraInfo.k[1], cameraInfo.k[2] },
                new double[] { cameraInfo.k[3], cameraInfo.k[4], cameraInfo.k[5] },
                new double[] { cameraInfo.k[6], cameraInfo.k[7], cameraInfo.k[8] },
            };

            var inverseK = MatrixExtensions.MatrixInverse(formatK);
            for (int i = 0; i < tex.width; i++)
            {
                for (int j = 0; j < tex.height; j++)
                {
                    var newPix = FindPixelProjection(i, j, cameraInfo.p, inverseK);
                    var color = tex.GetPixel((int)newPix.x, (int)newPix.y);
                    newTex.SetPixel(i, j, color);
                }
            }
            newTex.Apply();

            return newTex;
        }

        public static CompressedImageMsg ToCompressedImageMsg_JPG(this Texture2D tex, HeaderMsg header)
        {
            return new CompressedImageMsg(header, "jpeg", tex.EncodeToJPG());
        }

        public static CompressedImageMsg ToCompressedImageMsg_PNG(this Texture2D tex, HeaderMsg header)
        {
            return new CompressedImageMsg(header, "png", tex.EncodeToPNG());
        }

        /// <summary>
        /// Creates a new Texture2D that grabs a region of interest of a given texture, if applicable. Otherwise, an approximated empty texture with a highlighted region is returned.
        /// </summary>
        public static Texture2D RegionOfInterestTexture(this RegionOfInterestMsg message, Texture2D rawTex, int height = 0, int width = 0)
        {
            int mWidth = (int)message.width;
            int mHeight = (int)message.height;
            int x_off = ((int)message.x_offset == mWidth) ? 0 : (int)message.x_offset;
            int y_off = ((int)message.y_offset == mHeight) ? 0 : (int)message.y_offset;

            Texture2D overlay;
            if (rawTex == null)
            {
                // No texture provided, just return approximation
                if (width == 0 || height == 0)
                {
                    overlay = new Texture2D(x_off + mWidth + 10, y_off + mHeight + 10);
                }
                else
                {
                    overlay = new Texture2D(width, height);
                }

                // Initialize ROI color block
                Color[] colors = new Color[mHeight * mWidth];
                for (int i = 0; i < colors.Length; i++)
                {
                    colors[i] = Color.red;
                }

                overlay.SetPixels(x_off, y_off, mWidth, mHeight, colors);
            }
            else
            {
                // Crop out ROI from input texture
                overlay = new Texture2D(mWidth - x_off, mHeight - y_off, rawTex.format, true);
                overlay.SetPixels(0, 0, overlay.width, overlay.height, rawTex.GetPixels(x_off, 0, mWidth - x_off, mHeight - y_off));
            }
            overlay.Apply();
            return overlay;
        }

        public static ImageMsg ToImageMsg(this Texture2D tex, HeaderMsg header)
        {
            byte[] data = null;
            string encoding;
            int step;
            switch (tex.format)
            {
                case TextureFormat.RGB24:
                    data = new byte[tex.width * tex.height * 3];
                    tex.GetPixelData<byte>(0).CopyTo(data);
                    encoding = "rgb8";
                    step = 3 * tex.width;
                    ReverseInBlocks(data, tex.width * 3, tex.height);
                    break;
                case TextureFormat.RGBA32:
                    data = new byte[tex.width * tex.height * 4];
                    tex.GetPixelData<byte>(0).CopyTo(data);
                    encoding = "rgba8";
                    step = 4 * tex.width;
                    ReverseInBlocks(data, tex.width * 4, tex.height);
                    break;
                case TextureFormat.R8:
                    data = new byte[tex.width * tex.height];
                    tex.GetPixelData<byte>(0).CopyTo(data);
                    encoding = "8UC1";
                    step = 1 * tex.width;
                    ReverseInBlocks(data, tex.width, tex.height);
                    break;
                case TextureFormat.R16:
                    data = new byte[tex.width * tex.height * 2];
                    tex.GetPixelData<byte>(0).CopyTo(data);
                    encoding = "16UC1";
                    step = 2 * tex.width;
                    ReverseInBlocks(data, tex.width * 2, tex.height);
                    break;
                default:
                    Color32[] pixels = tex.GetPixels32();
                    data = new byte[pixels.Length * 4];
                    // although this is painfully slow, it does work... Surely there's a better way
                    int writeIdx = 0;
                    for (int Idx = 0; Idx < pixels.Length; ++Idx)
                    {
                        Color32 p = pixels[Idx];
                        data[writeIdx] = p.r;
                        data[writeIdx + 1] = p.g;
                        data[writeIdx + 2] = p.b;
                        data[writeIdx + 3] = p.a;
                        writeIdx += 4;
                    }
                    ReverseInBlocks(data, tex.width * 4, tex.height);
                    encoding = "rgba8";
                    step = 4 * tex.width;
                    break;
            }
            return new ImageMsg(header, height: (uint)tex.height, width: (uint)tex.width, encoding: encoding, is_bigendian: 0, step: (uint)step, data: data);
        }

        public static string ToLatLongString(this NavSatFixMsg message)
        {
            string lat = (message.latitude > 0) ? "ºN" : "ºS";
            string lon = (message.longitude > 0) ? "ºE" : "ºW";
            return $"{message.latitude}{lat} {message.longitude}{lon}";
        }
    }
}
