using System;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerOdometry : DrawingVisualFactory<MOdometry>
{
    public float thickness = 0.01f;
    public float lengthScale = 1.0f;
    public float sphereRadius = 1.0f;
    public GameObject origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MOdometry message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), origin, lengthScale, sphereRadius, thickness);
    }

    public override Action CreateGUI(MOdometry message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Child frame ID: {message.child_frame_id}");
        message.pose.pose.GUI("Pose:");
        message.twist.twist.GUI("Twist:");
    };
}
