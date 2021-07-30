using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerImu : StampedDrawingVisualFactory<ImuMsg>
{
    [SerializeField]
    public Color m_Color;
    [SerializeField]
    public float m_LengthScale = 1;
    [SerializeField]
    public float m_SphereRadius = 1;
    [SerializeField]
    public float m_Thickness = 0.01f;
    bool m_ViewAccel;
    bool m_ViewAngular;
    bool m_ViewOrientation;

    public override void Draw(BasicDrawing drawing, ImuMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_LengthScale, m_SphereRadius, m_Thickness);
    }

    public override Action CreateGUI(ImuMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.orientation.GUI("Orientation");
            message.angular_velocity.GUI("Angular velocity");
            message.linear_acceleration.GUI("Linear acceleration");
            MessageVisualizations.GUIGrid(message.orientation_covariance, 3, "Orientation covariance", ref m_ViewOrientation);
            MessageVisualizations.GUIGrid(message.angular_velocity_covariance, 3, "Angular velocity covariance", ref m_ViewAngular);
            MessageVisualizations.GUIGrid(message.linear_acceleration_covariance, 3, "Linear acceleration covariance", ref m_ViewAccel);
        };
    }
}
