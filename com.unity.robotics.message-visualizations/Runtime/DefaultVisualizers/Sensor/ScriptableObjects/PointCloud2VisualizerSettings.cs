using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

[CreateAssetMenu(fileName = "PointCloud2VisualizerSettings", menuName = "Robotics/Sensor/PointCloud2", order = 1)]
public class PointCloud2VisualizerSettings : VisualizerSettings<PointCloud2Msg>
{
    public enum ColorMode
    {
        HSV,
        RGB
    }

    public ColorMode colorMode;

    [HideInInspector]
    public string[] channels;

    public string m_XChannel = "x";
    public string m_YChannel = "y";
    public string m_ZChannel = "z";
    public string m_HueChannel = "x";
    public string m_RChannel = "r";
    public string m_GChannel = "g";
    public string m_BChannel = "b";
    public string m_SizeChannel = "x";

    public float[] m_HueRange = { 0, 31 };
    public float[] m_RRange = { -100, 100 };
    public float[] m_GRange = { -100, 100 };
    public float[] m_BRange = { -100, 100 };
    public float[] m_SizeRange = { 0, 100 };
    public float m_Size = 0.01f;

    public bool m_UseRgbChannel;
    public bool m_UseSizeChannel;

    public override void Draw(BasicDrawing drawing, PointCloud2Msg message, MessageMetadata meta)
    {
        PointCloudDrawing pointCloud = drawing.AddPointCloud((int)(message.data.Length / message.point_step));

        channels = message.fields.Select(field => field.name).ToArray();

        Dictionary<string, int> channelToIdx = new Dictionary<string, int>();
        for (int i = 0; i < message.fields.Length; i++)
        {
            channelToIdx.Add(message.fields[i].name, i);
        }

        TFFrame frame = TFSystem.instance.GetTransform(message.header);

        int xChannelOffset = (int)message.fields[channelToIdx[m_XChannel]].offset;
        int yChannelOffset = (int)message.fields[channelToIdx[m_YChannel]].offset;
        int zChannelOffset = (int)message.fields[channelToIdx[m_ZChannel]].offset;
        int rgbChannelOffset = (int)message.fields[channelToIdx[m_HueChannel]].offset;
        int rChannelOffset = (int)message.fields[channelToIdx[m_RChannel]].offset;
        int gChannelOffset = (int)message.fields[channelToIdx[m_GChannel]].offset;
        int bChannelOffset = (int)message.fields[channelToIdx[m_BChannel]].offset;
        int sizeChannelOffset = (int)message.fields[channelToIdx[m_SizeChannel]].offset;
        for (int i = 0; i < message.data.Length / message.point_step; i++)
        {
            int iPointStep = i * (int)message.point_step;
            var x = BitConverter.ToSingle(message.data, iPointStep + xChannelOffset);
            var y = BitConverter.ToSingle(message.data, iPointStep + yChannelOffset);
            var z = BitConverter.ToSingle(message.data, iPointStep + zChannelOffset);
            Vector3<FLU> localRosPoint = new Vector3<FLU>(x, y, z);
            Vector3 worldPoint = frame.TransformPoint(localRosPoint.toUnity);

            Color color = Color.white;

            // TODO: Parse type based on PointField?
            if (m_UseRgbChannel)
            {
                switch (colorMode)
                {
                    case ColorMode.HSV:
                        if (m_HueChannel.Length > 0)
                        {
                            int colC = BitConverter.ToInt16(message.data, (iPointStep + rgbChannelOffset));
                            color = Color.HSVToRGB(Mathf.InverseLerp(m_HueRange[0], m_HueRange[1], colC), 1, 1);
                        }
                        break;
                    case ColorMode.RGB:
                        if (m_RChannel.Length > 0 && m_GChannel.Length > 0 && m_BChannel.Length > 0)
                        {
                            var colR = Mathf.InverseLerp(m_RRange[0], m_RRange[1], BitConverter.ToSingle(message.data, iPointStep + rChannelOffset));
                            var r = Mathf.InverseLerp(0, 1, colR);

                            var colG = Mathf.InverseLerp(m_GRange[0], m_GRange[1], BitConverter.ToSingle(message.data, iPointStep + gChannelOffset));
                            var g = Mathf.InverseLerp(0, 1, colG);

                            var colB = Mathf.InverseLerp(m_BRange[0], m_BRange[1], BitConverter.ToSingle(message.data, iPointStep + bChannelOffset));
                            var b = Mathf.InverseLerp(0, 1, colB);
                            color = new Color(r, g, b, 1);
                        }
                        break;
                }
            }

            var radius = m_Size;

            if (m_UseSizeChannel)
            {
                var size = BitConverter.ToSingle(message.data, iPointStep + sizeChannelOffset);
                radius = Mathf.InverseLerp(m_SizeRange[0], m_SizeRange[1], size);
            }

            pointCloud.AddPoint(worldPoint, color, radius);
        }
        //pointCloud.Bake();
    }

    public override Action CreateGUI(PointCloud2Msg message, MessageMetadata meta)
    {
        var formatDict = new Dictionary<PointField_Format_Constants, List<string>>();

        foreach (var field in message.fields)
            if (formatDict.ContainsKey((PointField_Format_Constants)field.datatype))
                formatDict[(PointField_Format_Constants)field.datatype].Add(field.name);
            else
                formatDict.Add((PointField_Format_Constants)field.datatype, new List<string> { field.name });

        var formats = "";
        foreach (var f in formatDict)
            if (f.Value.Count > 0)
                formats += $"{f.Key}: {string.Join(", ", f.Value)}\n";

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nData length: {message.data.Length}\nPoint step: {message.point_step}\nRow step: {message.row_step}\nIs dense: {message.is_dense}");
            GUILayout.Label($"Channels:\n{formats}");
        };
    }

}
