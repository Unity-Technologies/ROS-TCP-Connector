//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Sensor
{
    public class MRegionOfInterest : Message
    {
        public const string k_RosMessageName = "sensor_msgs/RegionOfInterest";
        public override string RosMessageName => k_RosMessageName;

        //  This message is used to specify a region of interest within an image.
        // 
        //  When used to specify the ROI setting of the camera when the image was
        //  taken, the height and width fields should either match the height and
        //  width fields for the associated image; or height = width = 0
        //  indicates that the full resolution image was captured.
        public uint x_offset;
        //  Leftmost pixel of the ROI
        //  (0 if the ROI includes the left edge of the image)
        public uint y_offset;
        //  Topmost pixel of the ROI
        //  (0 if the ROI includes the top edge of the image)
        public uint height;
        //  Height of ROI
        public uint width;
        //  Width of ROI
        //  True if a distinct rectified ROI should be calculated from the "raw"
        //  ROI in this message. Typically this should be False if the full image
        //  is captured (ROI not used), and True if a subwindow is captured (ROI
        //  used).
        public bool do_rectify;

        public MRegionOfInterest()
        {
            this.x_offset = 0;
            this.y_offset = 0;
            this.height = 0;
            this.width = 0;
            this.do_rectify = false;
        }

        public MRegionOfInterest(uint x_offset, uint y_offset, uint height, uint width, bool do_rectify)
        {
            this.x_offset = x_offset;
            this.y_offset = y_offset;
            this.height = height;
            this.width = width;
            this.do_rectify = do_rectify;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.x_offset));
            listOfSerializations.Add(BitConverter.GetBytes(this.y_offset));
            listOfSerializations.Add(BitConverter.GetBytes(this.height));
            listOfSerializations.Add(BitConverter.GetBytes(this.width));
            listOfSerializations.Add(BitConverter.GetBytes(this.do_rectify));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.x_offset = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.y_offset = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.height = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.width = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.do_rectify = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

        public override string ToString()
        {
            return "MRegionOfInterest: " +
            "\nx_offset: " + x_offset.ToString() +
            "\ny_offset: " + y_offset.ToString() +
            "\nheight: " + height.ToString() +
            "\nwidth: " + width.ToString() +
            "\ndo_rectify: " + do_rectify.ToString();
        }

        [UnityEngine.RuntimeInitializeOnLoadMethod(UnityEngine.RuntimeInitializeLoadType.AfterAssembliesLoaded)]
        static void OnLoad()
        {
            MessageRegistry.Register<MRegionOfInterest>(k_RosMessageName);
        }
    }
}