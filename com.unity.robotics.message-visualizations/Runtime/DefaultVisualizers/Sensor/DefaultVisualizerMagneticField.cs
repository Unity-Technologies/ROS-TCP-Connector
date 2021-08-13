using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMagneticField : DrawingVisualFactory<MagneticFieldMsg>
{
    [SerializeField]
    Color m_Color;
    bool m_ViewCovariance;

    public override void Draw(BasicDrawing drawing, MagneticFieldMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta));
    }

    public static void Draw<C>(MagneticFieldMsg message, BasicDrawing drawing, Color color, float lengthScale = 1) where C : ICoordinateSpace, new()
    {
        drawing.DrawArrow(Vector3.zero, message.magnetic_field.From<C>() * lengthScale, color);
    }

    public override Action CreateGUI(MagneticFieldMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.magnetic_field.GUI("Magnetic field (Tesla)");
            MessageVisualizationUtils.GUIGrid(message.magnetic_field_covariance, 3, "Covariance", ref m_ViewCovariance);
        };
    }
}
