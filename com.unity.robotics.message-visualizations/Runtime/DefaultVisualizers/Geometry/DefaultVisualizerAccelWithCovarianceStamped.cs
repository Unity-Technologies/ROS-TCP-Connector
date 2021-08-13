using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerAccelWithCovarianceStamped : DrawingVisualFactory<AccelWithCovarianceStampedMsg>
{
    public float m_Thickness = 0.01f;
    public float m_LengthScale = 1.0f;
    public float m_SphereRadius = 1.0f;
    public GameObject m_Origin;
    [SerializeField]
    Color m_Color;
    bool m_ViewCovariance;

    public override void Draw(BasicDrawing drawing, AccelWithCovarianceStampedMsg message, MessageMetadata meta)
    {
        DefaultVisualizerAccelWithCovariance.Draw<FLU>(message.accel, drawing, SelectColor(m_Color, meta), m_Origin, m_LengthScale, m_SphereRadius, m_Thickness);
    }

    public override Action CreateGUI(AccelWithCovarianceStampedMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.accel.accel.GUI();
            MessageVisualizationUtils.GUIGrid(message.accel.covariance, 6, ref m_ViewCovariance);
        };
    }
}
