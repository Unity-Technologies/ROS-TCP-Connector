using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerLaserScan : BasicVisualizer<MLaserScan>
{
    public LaserScanVisualizerSettings m_Settings;

    public override void Draw(BasicDrawing drawing, MLaserScan message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, m_Settings);
    }

    public override Action CreateGUI(MLaserScan message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Angle min {message.angle_min}, max {message.angle_max}, increment {message.angle_increment}");
        GUILayout.Label($"Time between measurements {message.time_increment}; time between scans {message.scan_time}");
        GUILayout.Label($"Range min {message.range_min}, max {message.range_max}");
        GUILayout.Label(message.intensities.Length == 0 ? $"{message.ranges.Length} range entries (no intensity data)" : $"{message.ranges.Length} range and intensity entries");
    };
}
