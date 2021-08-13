using System;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerOdometry : DrawingVisualFactory<OdometryMsg>
{
    public float thickness = 0.01f;
    public float lengthScale = 1.0f;
    public float sphereRadius = 1.0f;
    public GameObject origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, OdometryMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), origin, lengthScale, sphereRadius, thickness);
    }

    public static void Draw<C>(OdometryMsg message, BasicDrawing drawing, Color color, GameObject origin, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
    {
        // TODO
        TFFrame frame = TFSystem.instance.GetTransform(message.header);
        Vector3 pos = frame.TransformPoint(message.pose.pose.position.From<C>());
        if (origin != null)
        {
            pos += origin.transform.position;
        }
        DefaultVisualizerPose.Draw<C>(message.pose.pose, drawing);
        DefaultVisualizerTwist.Draw<C>(message.twist.twist, drawing, color, pos, lengthScale, sphereRadius, thickness);
    }

    public override Action CreateGUI(OdometryMsg message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Child frame ID: {message.child_frame_id}");
        message.pose.pose.GUI("Pose:");
        message.twist.twist.GUI("Twist:");
    };
}
