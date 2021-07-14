using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.TransformManagement;
using UnityEngine;

[CreateAssetMenu(fileName = "PointCloudVisualizerSettings", menuName = "MessageVisualizations/Sensor/PointCloud", order = 1)]
public class PointCloudVisualizerSettings : VisualizerSettings<PointCloudMsg>
{
    public PointCloud2VisualizerSettings.ColorMode colorMode;
    public ChannelFloat32Msg[] channels;
    public string m_HueChannel = "";
    public string m_RChannel = "";
    public string m_GChannel = "";
    public string m_BChannel = "";
    public string m_SizeChannel = "";

    public float[] m_HueRange = { 0, 100 };
    public float[] m_RRange = { 0, 100 };
    public float[] m_GRange = { 0, 100 };
    public float[] m_BRange = { 0, 100 };
    public float[] m_SizeRange = { 0, 100 };
    public float m_Size = 0.05f;

    public bool m_UseRgbChannel;
    public bool m_UseSeparateRgb = true;
    public bool m_UseSizeChannel;

    public override Action CreateGUI(PointCloudMsg message, MessageMetadata meta)
    {
        var channelNames = string.Join(", ", message.channels.Select(i => i.name));

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Length of points: {message.points.Length}\nChannel names: {channelNames}");
        };
    }

    public override void Draw(BasicDrawing drawing, PointCloudMsg message, MessageMetadata meta)
    {
        PointCloudDrawing pointCloud = drawing.AddPointCloud();

        if (channels == null)
            channels = message.channels;

        pointCloud.SetCapacity(message.points.Length);
        TransformFrame frame = TransformManager.instance.GetTransform(message.header);

        Dictionary<string, int> channelToIdx = new Dictionary<string, int>();
        for (int i = 0; i < message.channels.Length; i++)
        {
            channelToIdx.Add(message.channels[i].name, i);
        }

        Color color = Color.white;
        for (int i = 0; i < message.points.Length; i++)
        {
            Vector3 worldPoint = frame.TransformPoint(message.points[i].From<FLU>());

            if (m_UseRgbChannel)
            {
                switch (colorMode)
                {
                    case PointCloud2VisualizerSettings.ColorMode.HSV:
                        if (m_HueChannel.Length > 0)
                        {
                            float colC = message.channels[channelToIdx[m_HueChannel]].values[i];
                            color = Color.HSVToRGB(Mathf.InverseLerp(m_HueRange[0], m_HueRange[1], colC), 1, 1);
                        }
                        break;
                    case PointCloud2VisualizerSettings.ColorMode.RGB:
                        if (m_UseSeparateRgb)
                        {
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
                        }
                        else
                        {
                            // uint8 (R,G,B) values packed into the least significant 24 bits, in order.
                            byte[] rgb = BitConverter.GetBytes(message.channels[channelToIdx[m_HueChannel]].values[i]);

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

            pointCloud.AddPoint(worldPoint, color, radius);
        }
        pointCloud.Bake();
    }
}
