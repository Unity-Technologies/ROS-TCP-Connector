using System;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PathDefaultVisualizer : DrawingVisualizer<PathMsg>
{
    [SerializeField]
    float m_Thickness = 0.1f;
    [SerializeField]
    Color m_Color;
    [SerializeField]
    Vector3 m_Offset = Vector3.zero;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;

    public override void Draw(Drawing3d drawing, PathMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), m_Thickness, m_Offset);
    }

    public static void Draw<C>(PathMsg message, Drawing3d drawing, Color color, float thickness = 0.1f, Vector3 offset = default(Vector3))
        where C : ICoordinateSpace, new()
    {
        drawing.DrawPath(message.poses.Select(pose => pose.pose.position.From<C>() + offset), color, thickness);
    }

    public override Action CreateGUI(PathMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            foreach (PoseStampedMsg pose in message.poses)
            {
                pose.header.GUI();
                pose.pose.GUI();
            }
        };
    }
}
