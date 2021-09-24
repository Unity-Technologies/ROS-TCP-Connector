using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

[CreateAssetMenu(fileName = "PointCloud2VisualizerSettings", menuName = "Robotics/Sensor Visualizers/PointCloud2", order = 1)]
public class PointCloud2VisualizerSettings : VisualizerSettingsGeneric<PointCloud2Msg>
{
    public enum ColorMode
    {
        HSV,
        SeparateRGB,
        CombinedRGB,
    }

    [HideInInspector, SerializeField]
    ColorMode m_ColorModeSetting;
    public ColorMode ColorModeSetting { get => m_ColorModeSetting; set => m_ColorModeSetting = value; }
    public string[] Channels { get => m_Channels; set => m_Channels = value; }
    string[] m_Channels;

    public string XChannel { get => m_XChannel; set => m_XChannel = value; }
    string m_XChannel = "x";
    public string YChannel { get => m_YChannel; set => m_YChannel = value; }
    string m_YChannel = "y";
    public string ZChannel { get => m_ZChannel; set => m_ZChannel = value; }
    string m_ZChannel = "z";
    public string HueChannel { get => m_HueChannel; set => m_HueChannel = value; }
    string m_HueChannel = "";
    public string RgbChannel { get => m_RgbChannel; set => m_RgbChannel = value; }
    string m_RgbChannel = "rgb";
    public string RChannel { get => m_RChannel; set => m_RChannel = value; }
    string m_RChannel = "";
    public string GChannel { get => m_GChannel; set => m_GChannel = value; }
    string m_GChannel = "";
    public string BChannel { get => m_BChannel; set => m_BChannel = value; }
    string m_BChannel = "";
    public string SizeChannel { get => m_SizeChannel; set => m_SizeChannel = value; }
    string m_SizeChannel = "";

    public float[] HueRange { get => m_HueRange; set => m_HueRange = value; }
    float[] m_HueRange = { 0, 100 };
    public float[] RRange { get => m_RRange; set => m_RRange = value; }
    float[] m_RRange = { 0, 100 };
    public float[] GRange { get => m_GRange; set => m_GRange = value; }
    float[] m_GRange = { 0, 100 };
    public float[] BRange { get => m_BRange; set => m_BRange = value; }
    float[] m_BRange = { 0, 100 };
    public float[] SizeRange { get => m_SizeRange; set => m_SizeRange = value; }
    float[] m_SizeRange = { 0, 100 };
    public float Size { get => m_Size; set => m_Size = value; }
    float m_Size = 0.05f;

    public bool UseRgbChannel { get => m_UseRgbChannel; set => m_UseRgbChannel = value; }
    bool m_UseRgbChannel = true;
    public bool UseSizeChannel { get => m_UseSizeChannel; set => m_UseSizeChannel = value; }
    bool m_UseSizeChannel = true;

    public override void Draw(Drawing3d drawing, PointCloud2Msg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        var pointCloud = drawing.AddPointCloud((int)(message.data.Length / message.point_step));

        Channels = message.fields.Select(field => field.name).ToArray();

        Dictionary<string, int> channelToIdx = new Dictionary<string, int>();
        for (int i = 0; i < message.fields.Length; i++)
        {
            channelToIdx.Add(message.fields[i].name, i);
        }

        TFFrame frame = TFSystem.instance.GetTransform(message.header);

        Func<int, Color> colorGenerator = (int iPointStep) => Color.white;

        if (m_UseRgbChannel)
        {
            switch (ColorModeSetting)
            {
                case ColorMode.HSV:
                    if (m_HueChannel.Length > 0)
                    {
                        int hueChannelOffset = (int)message.fields[channelToIdx[m_HueChannel]].offset;
                        colorGenerator = (int iPointStep) =>
                        {
                            int colC = BitConverter.ToInt16(message.data, (iPointStep + hueChannelOffset));
                            return Color.HSVToRGB(Mathf.InverseLerp(m_HueRange[0], m_HueRange[1], colC), 1, 1);
                        };
                    }
                    break;
                case ColorMode.SeparateRGB:
                    if (m_RChannel.Length > 0 && m_GChannel.Length > 0 && m_BChannel.Length > 0)
                    {
                        int rChannelOffset = (int)message.fields[channelToIdx[m_RChannel]].offset;
                        int gChannelOffset = (int)message.fields[channelToIdx[m_GChannel]].offset;
                        int bChannelOffset = (int)message.fields[channelToIdx[m_BChannel]].offset;
                        colorGenerator = (int iPointStep) =>
                        {
                            var colR = Mathf.InverseLerp(m_RRange[0], m_RRange[1], BitConverter.ToSingle(message.data, iPointStep + rChannelOffset));
                            var colG = Mathf.InverseLerp(m_GRange[0], m_GRange[1], BitConverter.ToSingle(message.data, iPointStep + gChannelOffset));
                            var colB = Mathf.InverseLerp(m_BRange[0], m_BRange[1], BitConverter.ToSingle(message.data, iPointStep + bChannelOffset));
                            return new Color(colR, colG, colB, 1);
                        };
                    }
                    break;
                case ColorMode.CombinedRGB:
                    if (m_RgbChannel.Length > 0)
                    {
                        int rgbChannelOffset = (int)message.fields[channelToIdx[m_RgbChannel]].offset;
                        colorGenerator = (int iPointStep) => new Color32
                        (
                            message.data[iPointStep + rgbChannelOffset + 2],
                            message.data[iPointStep + rgbChannelOffset + 1],
                            message.data[iPointStep + rgbChannelOffset],
                            255
                        );
                    }
                    break;
            }
        }

        int xChannelOffset = (int)message.fields[channelToIdx[m_XChannel]].offset;
        int yChannelOffset = (int)message.fields[channelToIdx[m_YChannel]].offset;
        int zChannelOffset = (int)message.fields[channelToIdx[m_ZChannel]].offset;
        int sizeChannelOffset = 0;
        bool useSizeChannel = m_UseSizeChannel && m_SizeChannel != "";
        if (useSizeChannel)
            sizeChannelOffset = (int)message.fields[channelToIdx[m_SizeChannel]].offset;
        int maxI = message.data.Length / (int)message.point_step;
        for (int i = 0; i < maxI; i++)
        {
            int iPointStep = i * (int)message.point_step;
            var x = BitConverter.ToSingle(message.data, iPointStep + xChannelOffset);
            var y = BitConverter.ToSingle(message.data, iPointStep + yChannelOffset);
            var z = BitConverter.ToSingle(message.data, iPointStep + zChannelOffset);
            Vector3<FLU> rosPoint = new Vector3<FLU>(x, y, z);
            Vector3 unityPoint = rosPoint.toUnity;

            Color color = colorGenerator(iPointStep);

            float radius;
            if (useSizeChannel)
            {
                var size = BitConverter.ToSingle(message.data, iPointStep + sizeChannelOffset);
                radius = Mathf.InverseLerp(m_SizeRange[0], m_SizeRange[1], size) * m_Size;
            }
            else
            {
                radius = m_Size;
            }

            pointCloud.AddPoint(unityPoint, color, radius);
        }
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
