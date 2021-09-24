using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

[CreateAssetMenu(fileName = "PointCloudVisualizerSettings", menuName = "Robotics/Sensor Visualizers/PointCloud", order = 1)]
public class PointCloudVisualizerSettings : VisualizerSettingsGeneric<PointCloudMsg>
{
    [HideInInspector, SerializeField]
    PointCloud2VisualizerSettings.ColorMode m_ColorMode;
    public PointCloud2VisualizerSettings.ColorMode ColorMode { get => m_ColorMode; set => m_ColorMode = value; }

    public string[] Channels { get => m_Channels; set => m_Channels = value; }
    string[] m_Channels;
    public string HueChannel { get => m_HueChannel; set => m_HueChannel = value; }
    string m_HueChannel = "";
    public string RgbChannel { get => m_RgbChannel; set => m_RgbChannel = value; }
    string m_RgbChannel = "";
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
    public bool UseSeparateRgb { get => m_UseSeparateRgb; set => m_UseSeparateRgb = value; }
    bool m_UseSeparateRgb = true;
    public bool UseSizeChannel { get => m_UseSizeChannel; set => m_UseSizeChannel = value; }
    bool m_UseSizeChannel = true;

    public override Action CreateGUI(PointCloudMsg message, MessageMetadata meta)
    {
        var channelNames = string.Join(", ", message.channels.Select(i => i.name));

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Length of points: {message.points.Length}\nChannel names: {channelNames}");
        };
    }

    public override void Draw(Drawing3d drawing, PointCloudMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        PointCloudDrawing pointCloud = drawing.AddPointCloud();
        Channels = message.channels.Select(field => field.name).ToArray();

        pointCloud.SetCapacity(message.points.Length);

        Dictionary<string, int> channelToIdx = new Dictionary<string, int>();
        for (int i = 0; i < message.channels.Length; i++)
        {
            channelToIdx.Add(message.channels[i].name, i);
        }

        Color color = Color.white;
        for (int i = 0; i < message.points.Length; i++)
        {
            Vector3 point = message.points[i].From<FLU>();

            if (m_UseRgbChannel)
            {
                switch (m_ColorMode)
                {
                    case PointCloud2VisualizerSettings.ColorMode.HSV:
                        if (m_HueChannel.Length > 0)
                        {
                            float colC = message.channels[channelToIdx[m_HueChannel]].values[i];
                            color = Color.HSVToRGB(Mathf.InverseLerp(m_HueRange[0], m_HueRange[1], colC), 1, 1);
                        }
                        break;
                    case PointCloud2VisualizerSettings.ColorMode.SeparateRGB:
                        if (m_RChannel.Length > 0 && m_GChannel.Length > 0 && m_BChannel.Length > 0)
                        {
                            var colR = Mathf.InverseLerp(m_RRange[0], m_RRange[1], message.channels[channelToIdx[m_RChannel]].values[i]);
                            var r = Mathf.InverseLerp(0, 1, colR);

                            var colG = Mathf.InverseLerp(m_GRange[0], m_GRange[1], message.channels[channelToIdx[m_GChannel]].values[i]);
                            var g = Mathf.InverseLerp(0, 1, colG);

                            var colB = Mathf.InverseLerp(m_BRange[0], m_BRange[1], message.channels[channelToIdx[m_BChannel]].values[i]);
                            var b = Mathf.InverseLerp(0, 1, colB);
                            color = new Color(r, g, b, 1);
                        }
                        break;
                    case PointCloud2VisualizerSettings.ColorMode.CombinedRGB:
                        // uint8 (R,G,B) values packed into the least significant 24 bits, in order.
                        {
                            byte[] rgb = BitConverter.GetBytes(message.channels[channelToIdx[m_RgbChannel]].values[i]);

                            var r = Mathf.InverseLerp(0, 1, BitConverter.ToSingle(rgb, 0));
                            var g = Mathf.InverseLerp(0, 1, BitConverter.ToSingle(rgb, 8));
                            var b = Mathf.InverseLerp(0, 1, BitConverter.ToSingle(rgb, 16));
                            color = new Color(r, g, b, 1);
                        }
                        break;
                }
            }

            var radius = m_Size;
            if (m_UseSizeChannel && m_SizeChannel.Length > 0)
            {
                var size = message.channels[channelToIdx[m_SizeChannel]].values[i];
                radius = Mathf.InverseLerp(m_SizeRange[0], m_SizeRange[1], size);
            }

            pointCloud.AddPoint(point, color, radius);
        }
        pointCloud.Bake();
    }
}
