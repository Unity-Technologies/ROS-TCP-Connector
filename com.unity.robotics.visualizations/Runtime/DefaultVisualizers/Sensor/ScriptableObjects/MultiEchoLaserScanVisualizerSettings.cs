using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

[CreateAssetMenu(fileName = "MultiEchoLaserScanVisualizerSettings", menuName = "Robotics/Sensor Visualizers/MultiEchoLaserScan", order = 1)]
public class MultiEchoLaserScanVisualizerSettings : VisualizerSettingsGeneric<MultiEchoLaserScanMsg>
{
    [SerializeField]
    bool m_UseIntensitySize;
    public bool UseIntensitySize { get => m_UseIntensitySize; set => m_UseIntensitySize = value; }
    [HideInInspector, SerializeField]
    float m_PointRadius = 0.05f;
    public float PointRadius { get => m_PointRadius; set => m_PointRadius = value; }
    [HideInInspector, SerializeField]
    float[] m_SizeRange = { 0, 100 };
    public float[] SizeRange { get => m_SizeRange; set => m_SizeRange = value; }

    public override void Draw(Drawing3d drawing, MultiEchoLaserScanMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        MultiEchoLaserScanDefaultVisualizer.Draw<FLU>(message, drawing, this);
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
