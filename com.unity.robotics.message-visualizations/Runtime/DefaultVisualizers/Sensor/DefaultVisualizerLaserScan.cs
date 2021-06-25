using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerLaserScan : DrawingVisualFactory<MLaserScan>
{
    public LaserScanVisualizerSettings settings;
    
    public override void Start()
    {
        if (settings != null)
        {
            if (settings.topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<MLaserScan>(this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(settings.topic, this, Priority);
            }       
        }
    }

    public override void Draw(BasicDrawing drawing, MLaserScan message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, settings);
    }

    public override Action CreateGUI(MLaserScan message, MessageMetadata meta) => () =>
    {
        message.header.GUI();   
        GUILayout.Label($"Angle min {message.angle_min}, max {message.angle_max}, increment {message.angle_increment}");
        GUILayout.Label($"Time between measurements {message.time_increment}; time between scans {message.scan_time}");
        GUILayout.Label($"Range min {message.range_min}, max {message.range_max}");
        GUILayout.Label(message.intensities.Length == 0 ? $"{message.ranges.Length} range entries (no intensity data)" : $"{message.ranges.Length} range and intensity entries");
    };
}
