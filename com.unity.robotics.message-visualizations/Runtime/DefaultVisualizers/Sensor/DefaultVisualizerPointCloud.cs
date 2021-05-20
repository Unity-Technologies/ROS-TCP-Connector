using RosMessageTypes.Sensor;
using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPointCloud : BasicVisualizer<MPointCloud>
{
    public PointCloudVisualizerSettings m_Settings;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MPointCloud message, MessageMetadata meta)
    {
        if (m_Settings.channels == null)
            m_Settings.channels = message.channels;
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Settings);
    }

    public override Action CreateGUI(MPointCloud message, MessageMetadata meta, BasicDrawing drawing) 
    {
        string channelNames = String.Join(", ", message.channels.Select(i => i.name));

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Length of points: {message.points.Length}\nChannel names: {channelNames}");
        };
    }
}
