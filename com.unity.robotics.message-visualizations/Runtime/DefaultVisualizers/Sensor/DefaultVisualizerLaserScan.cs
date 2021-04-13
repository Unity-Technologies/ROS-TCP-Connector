using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerLaserScan : BasicVisualizer<MLaserScan>
{
    public float pointRadius = 0.05f;
    PointCloudDrawing pointCloud;

    public override void Draw(BasicDrawing drawing, MLaserScan message, MessageMetadata meta, Color color, string label)
    {
        // Big performance hack, reuse the same pointcloud for each drawing to reduce the amount of GC we're doing.
        // TODO: pool these
        if (pointCloud == null)
        {
            drawing = BasicDrawing.Create();
            pointCloud = drawing.AddPointCloud(message.ranges.Length);
        }
        pointCloud.SetCapacity(message.ranges.Length);
        TFFrame frame = TFSystem.instance.GetTransform(message.header);
        float angle = message.angle_min;
        for (int Idx = 0; Idx < message.ranges.Length; ++Idx)
        {
            Vector3 localPoint = Quaternion.Euler(0, -Mathf.Rad2Deg * angle, 0) * Vector3.forward * message.ranges[Idx];
            Vector3 worldPoint = frame.TransformPoint(localPoint);
            Color c = Color.HSVToRGB(Mathf.InverseLerp(message.range_min, message.range_max, message.ranges[Idx]), 1, 1);
            pointCloud.AddPoint(worldPoint, c, pointRadius);
            angle += message.angle_increment;
        }
        pointCloud.Bake();
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
