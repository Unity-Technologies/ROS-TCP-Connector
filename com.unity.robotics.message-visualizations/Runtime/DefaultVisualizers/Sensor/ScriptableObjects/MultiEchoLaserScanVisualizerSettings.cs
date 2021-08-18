using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

[CreateAssetMenu(fileName = "MultiEchoLaserScanVisualizerSettings", menuName = "MessageVisualizations/Sensor/MultiEchoLaserScan", order = 1)]
public class MultiEchoLaserScanVisualizerSettings : BaseVisualizerSettings<MultiEchoLaserScanMsg>
{
    [SerializeField]
    bool m_UseIntensitySize;
    public bool UseIntensitySize { get => m_UseIntensitySize; set => m_UseIntensitySize = value; }
    [SerializeField]
    float m_PointRadius = 0.05f;
    public float PointRadius { get => m_PointRadius; set => m_PointRadius = value; }
    [SerializeField]
    float[] m_SizeRange = { 0, 100 };
    public float[] SizeRange { get => m_SizeRange; set => m_SizeRange = value; }

    public override void Draw(BasicDrawing drawing, MultiEchoLaserScanMsg message, MessageMetadata meta)
    {
        MultiEchoLaserScanDefaultVisualizer.Draw<FLU>(message, drawing, this);
    }

    /* FNF changes LRCTEMP
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        var pointCloud = drawing.AddPointCloud(message.ranges.Length);
        pointCloud.SetCapacity(message.ranges.Length * message.ranges[0].echoes.Length);

        // negate the angle because ROS coordinates are right-handed, unity coordinates are left-handed
        float angle = -message.angle_min;
        for (int i = 0; i < message.ranges.Length; i++)
        {
            var echoes = message.ranges[i].echoes;
            for (int j = 0; j < echoes.Length; j++)
            {
                Vector3 point = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0) * Vector3.forward * echoes[j];
                Color c = Color.HSVToRGB(Mathf.InverseLerp(message.range_min, message.range_max, echoes[j]), 1, 1);

                var radius = m_PointRadius;

                if (message.intensities.Length > 0 && m_UseIntensitySize)
                {
                    radius = Mathf.InverseLerp(m_SizeRange[0], m_SizeRange[1], message.intensities[i].echoes[j]);
                }

                pointCloud.AddPoint(point, c, radius);
            }
            angle -= message.angle_increment;
        }
        pointCloud.Bake();
    }*/

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
