using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerAccel : DrawingVisualFactory<AccelMsg>
{
    public float m_Thickness = 0.01f;
    public float m_LengthScale = 1.0f;
    public float m_SphereRadius = 1.0f;
    public GameObject m_Origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, AccelMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), m_Origin, m_LengthScale, m_SphereRadius, m_Thickness);
    }

    public static void Draw<C>(AccelMsg message, BasicDrawing drawing, Color color, GameObject origin, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
    {
        Vector3 originPos = (origin == null) ? Vector3.zero : origin.transform.position;
        drawing.DrawArrow(originPos, originPos + message.linear.From<C>() * lengthScale, color, thickness);
        MessageVisualizationUtils.DrawAngularVelocityArrow(drawing, message.angular.From<C>(), originPos, color, sphereRadius, thickness);
    }

    public override Action CreateGUI(AccelMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
