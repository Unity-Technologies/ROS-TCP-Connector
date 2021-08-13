using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerAccelWithCovariance : DrawingVisualFactory<AccelWithCovarianceMsg>
{
    public float m_Thickness = 0.01f;
    public float m_LengthScale = 1.0f;
    public float m_SphereRadius = 1.0f;
    public GameObject m_Origin;
    [SerializeField]
    Color m_Color;
    bool m_ViewCovariance;

    public override void Draw(BasicDrawing drawing, AccelWithCovarianceMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), m_Origin, m_LengthScale, m_SphereRadius, m_Thickness);
    }

    public static void Draw<C>(AccelWithCovarianceMsg message, BasicDrawing drawing, Color color, GameObject origin, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
    {
        DefaultVisualizerAccel.Draw<FLU>(message.accel, drawing, color, origin, lengthScale, sphereRadius, thickness);
    }

    public override Action CreateGUI(AccelWithCovarianceMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.accel.GUI();
            MessageVisualizationUtils.GUIGrid(message.covariance, 6, ref m_ViewCovariance);
        };
    }
}
