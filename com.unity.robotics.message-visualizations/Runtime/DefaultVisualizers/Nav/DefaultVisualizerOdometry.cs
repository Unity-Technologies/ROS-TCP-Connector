using System;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerOdometry : DrawingStampedVisualFactory<OdometryMsg>
{
    public float thickness = 0.01f;
    public float lengthScale = 1.0f;
    public float sphereRadius = 1.0f;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, OdometryMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), lengthScale, sphereRadius, thickness);
    }

    public override Action CreateGUI(OdometryMsg message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Child frame ID: {message.child_frame_id}");
        message.pose.pose.GUI("Pose:");
        message.twist.twist.GUI("Twist:");
    };
}
