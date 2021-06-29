using System;
using System.Linq;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPointCloud : DrawingVisualFactory<PointCloudMsg>
{
    public PointCloudVisualizerSettings m_Settings;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, PointCloudMsg message, MessageMetadata meta)
    {
        if (m_Settings.channels == null)
            m_Settings.channels = message.channels;
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Settings);
    }

    public override Action CreateGUI(PointCloudMsg message, MessageMetadata meta)
    {
        var channelNames = string.Join(", ", message.channels.Select(i => i.name));

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Length of points: {message.points.Length}\nChannel names: {channelNames}");
        };
    }
}
