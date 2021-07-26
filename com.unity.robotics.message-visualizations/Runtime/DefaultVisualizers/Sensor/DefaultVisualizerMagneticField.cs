using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMagneticField : DrawingStampedVisualFactory<MagneticFieldMsg>
{
    [SerializeField]
    Color m_Color;
    bool m_ViewCovariance;

    public override void Draw(BasicDrawing drawing, MagneticFieldMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta));
    }

    public override Action CreateGUI(MagneticFieldMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.magnetic_field.GUI("Magnetic field (Tesla)");
            MessageVisualizations.GUIGrid(message.magnetic_field_covariance, 3, "Covariance", ref m_ViewCovariance);
        };
    }
}
