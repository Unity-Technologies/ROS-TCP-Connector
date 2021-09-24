using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class MultiEchoLaserScanDefaultVisualizer : DrawingVisualizerWithSettings<MultiEchoLaserScanMsg, MultiEchoLaserScanVisualizerSettings>
{
    public override string DefaultScriptableObjectPath => "VisualizerSettings/MultiEchoLaserScanVisualizerSettings";

    public override void Draw(Drawing3d drawing, MultiEchoLaserScanMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, Settings);
    }

    public static void Draw<C>(MultiEchoLaserScanMsg message, Drawing3d drawing, MultiEchoLaserScanVisualizerSettings settings)
        where C : ICoordinateSpace, new()
    {
        Draw<C>(message, drawing.AddPointCloud(message.ranges.Length), settings);
    }

    public static void Draw<C>(MultiEchoLaserScanMsg message, PointCloudDrawing pointCloud, MultiEchoLaserScanVisualizerSettings settings) where C : ICoordinateSpace, new()
    {
        pointCloud.SetCapacity(message.ranges.Length * message.ranges[0].echoes.Length);
        // negate the angle because ROS coordinates are right-handed, unity coordinates are left-handed
        float angle = -message.angle_min;
        // foreach(MLaserEcho echo in message.ranges)
        for (int i = 0; i < message.ranges.Length; i++)
        {
            var echoes = message.ranges[i].echoes;
            // foreach (float range in echo.echoes)
            for (int j = 0; j < echoes.Length; j++)
            {
                Vector3 point = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0) * Vector3.forward * echoes[j];
                Color c = Color.HSVToRGB(Mathf.InverseLerp(message.range_min, message.range_max, echoes[j]), 1, 1);

                var radius = settings.PointRadius;

                if (message.intensities.Length > 0 && settings.UseIntensitySize)
                {
                    radius = Mathf.InverseLerp(settings.SizeRange[0], settings.SizeRange[1], message.intensities[i].echoes[j]);
                }

                pointCloud.AddPoint(point, c, radius);
            }
            angle -= message.angle_increment;
        }
    }
}
