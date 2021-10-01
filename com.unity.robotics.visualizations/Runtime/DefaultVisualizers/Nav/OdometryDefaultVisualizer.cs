using System;
using RosMessageTypes.Nav;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class OdometryDefaultVisualizer : DrawingVisualizer<OdometryMsg>
{
    public float thickness = 0.01f;
    public float lengthScale = 1.0f;
    public float sphereRadius = 1.0f;
    [SerializeField]
    Color m_Color;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;

    public override void Draw(Drawing3d drawing, OdometryMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), lengthScale, sphereRadius, thickness);
    }

    public static void Draw<C>(OdometryMsg message, Drawing3d drawing, Color color, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
    {
        PoseDefaultVisualizer.Draw<C>(message.pose.pose, drawing);
        TwistDefaultVisualizer.Draw<C>(message.twist.twist, drawing, color, message.pose.pose.position.From<C>(), lengthScale, sphereRadius, thickness);
    }

    public override Action CreateGUI(OdometryMsg message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Child frame ID: {message.child_frame_id}");
        message.pose.pose.GUI("Pose:");
        message.twist.twist.GUI("Twist:");
    };
}
