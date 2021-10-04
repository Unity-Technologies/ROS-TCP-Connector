using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

[CreateAssetMenu(fileName = "LaserScanVisualizerSettings", menuName = "Robotics/Sensor Visualizers/LaserScan", order = 1)]
public class LaserScanVisualizerSettings : VisualizerSettingsGeneric<LaserScanMsg>
{
    [SerializeField]
    bool m_UseIntensitySize;
    public bool UseIntensitySize { get => m_UseIntensitySize; set => m_UseIntensitySize = value; }
    [HideInInspector, SerializeField]
    float m_PointRadius = 0.05f;
    public float PointRadius { get => m_PointRadius; set => m_PointRadius = value; }
    [HideInInspector, SerializeField]
    float m_MaxIntensity = 100.0f;
    public float MaxIntensity { get => m_MaxIntensity; set => m_MaxIntensity = value; }

    public enum ColorModeType
    {
        Distance,
        Intensity,
        Angle,
    }

    [SerializeField]
    ColorModeType m_ColorMode;
    public ColorModeType ColorMode { get => m_ColorMode; set => m_ColorMode = value; }

    public override void Draw(Drawing3d drawing, LaserScanMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);

        PointCloudDrawing pointCloud = drawing.AddPointCloud(message.ranges.Length);
        // negate the angle because ROS coordinates are right-handed, unity coordinates are left-handed
        float angle = -message.angle_min;
        ColorModeType mode = m_ColorMode;
        if (mode == ColorModeType.Intensity && message.intensities.Length != message.ranges.Length)
            mode = ColorModeType.Distance;
        for (int i = 0; i < message.ranges.Length; i++)
        {
            Vector3 point = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0) * Vector3.forward * message.ranges[i];

            Color32 c = Color.white;
            switch (mode)
            {
                case ColorModeType.Distance:
                    c = Color.HSVToRGB(Mathf.InverseLerp(message.range_min, message.range_max, message.ranges[i]), 1, 1);
                    break;
                case ColorModeType.Intensity:
                    c = new Color(1, message.intensities[i] / m_MaxIntensity, 0, 1);
                    break;
                case ColorModeType.Angle:
                    c = Color.HSVToRGB((1 + angle / (Mathf.PI * 2)) % 1, 1, 1);
                    break;
            }

            float radius = m_PointRadius;
            if (m_UseIntensitySize && message.intensities.Length > 0)
            {
                radius = Mathf.InverseLerp(0, m_MaxIntensity, message.intensities[i]);
            }
            pointCloud.AddPoint(point, c, radius);

            angle -= message.angle_increment;
        }
        pointCloud.Bake();
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
