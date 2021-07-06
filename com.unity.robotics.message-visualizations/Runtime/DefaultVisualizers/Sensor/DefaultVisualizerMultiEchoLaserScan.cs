using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMultiEchoLaserScan : DrawingVisualFactory<MultiEchoLaserScanMsg>
{
    public MultiEchoLaserScanVisualizerSettings settings;
    
    public override void Start()
    {
        if (settings != null)
        {
            m_Topic = settings.topic;
        }
        base.Start();
    }

    public override void Draw(BasicDrawing drawing, MultiEchoLaserScanMsg message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, settings);
    }

    public override Action CreateGUI(MultiEchoLaserScanMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Angle min, max: [{message.angle_min}, {message.angle_max}] (rad)\nIncrement: {message.angle_increment} (rad)");
            GUILayout.Label($"Time between measurements: {message.time_increment} (s)\nTime between scans: {message.scan_time} (s)");
            GUILayout.Label($"Range min, max: [{message.range_min}, {message.range_max}] (m)");
        };
    }
}
