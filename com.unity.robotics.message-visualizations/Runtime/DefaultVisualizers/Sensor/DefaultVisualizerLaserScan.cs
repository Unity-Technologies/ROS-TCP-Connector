using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerLaserScan : DrawingVisualFactory<LaserScanMsg>
{
    public LaserScanVisualizerSettings m_Settings;

    public override void Draw(BasicDrawing drawing, LaserScanMsg message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, m_Settings);
    }

    public override Action CreateGUI(LaserScanMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Angle min {message.angle_min}, max {message.angle_max}, increment {message.angle_increment}");
            GUILayout.Label($"Time between measurements {message.time_increment}; time between scans {message.scan_time}");
            GUILayout.Label($"Range min {message.range_min}, max {message.range_max}");
            GUILayout.Label(message.intensities.Length == 0 ? $"{message.ranges.Length} range entries (no intensity data)" : $"{message.ranges.Length} range and intensity entries");
        };
    }
}
