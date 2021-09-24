using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class MagneticFieldDefaultVisualizer : DrawingVisualizer<MagneticFieldMsg>
{
    [SerializeField]
    Color m_Color;
    bool m_ViewCovariance;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;

    public override void Draw(Drawing3d drawing, MagneticFieldMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta));
    }

    public static void Draw<C>(MagneticFieldMsg message, Drawing3d drawing, Color color, float lengthScale = 1) where C : ICoordinateSpace, new()
    {
        drawing.DrawArrow(Vector3.zero, message.magnetic_field.From<C>() * lengthScale, color);
    }

    public override Action CreateGUI(MagneticFieldMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.magnetic_field.GUI("Magnetic field (Tesla)");
            VisualizationUtils.GUIGrid(message.magnetic_field_covariance, 3, "Covariance", ref m_ViewCovariance);
        };
    }
}
