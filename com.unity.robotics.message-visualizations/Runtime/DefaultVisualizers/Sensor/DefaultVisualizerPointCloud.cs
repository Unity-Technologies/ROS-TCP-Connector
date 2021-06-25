using RosMessageTypes.Sensor;
using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPointCloud : DrawingVisualFactory<MPointCloud>
{
    public PointCloudVisualizerSettings settings;
    [SerializeField]
    Color m_Color;
    
    public override void Start()
    {
        if (settings != null)
        {
            if (settings.topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<MPointCloud>(this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(settings.topic, this, Priority);
            }       
        }
    }

    public override void Draw(BasicDrawing drawing, MPointCloud message, MessageMetadata meta)
    {
        if (settings.channels == null)
            settings.channels = message.channels;
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), settings);
    }

    public override Action CreateGUI(MPointCloud message, MessageMetadata meta) 
    {
        string channelNames = string.Join(", ", message.channels.Select(i => i.name));

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Length of points: {message.points.Length}\nChannel names: {channelNames}");
        };
    }
}
