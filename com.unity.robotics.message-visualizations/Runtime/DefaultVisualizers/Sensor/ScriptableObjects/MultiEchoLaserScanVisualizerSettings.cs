using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

[CreateAssetMenu(fileName = "MultiEchoLaserScanVisualizerSettings", menuName = "MessageVisualizations/Sensor/MultiEchoLaserScan", order = 1)]
public class MultiEchoLaserScanVisualizerSettings : VisualizerSettings<MultiEchoLaserScanMsg>
{
    public bool m_UseIntensitySize;
    public float m_PointRadius = 0.05f;
    public float[] m_SizeRange = { 0, 100 };

    public override void Draw(BasicDrawing drawing, MultiEchoLaserScanMsg message, MessageMetadata meta)
    {
        DefaultVisualizerMultiEchoLaserScan.Draw<FLU>(message, drawing, this);
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
