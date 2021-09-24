using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class ImuDefaultVisualizer : DrawingVisualizer<ImuMsg>
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
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;

    public override void Draw(Drawing3d drawing, ImuMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), m_LengthScale, m_SphereRadius, m_Thickness);
    }

    public static void Draw<C>(ImuMsg message, Drawing3d drawing, Color color, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
    {
        QuaternionDefaultVisualizer.Draw<C>(message.orientation, drawing);
        drawing.DrawArrow(Vector3.zero, message.linear_acceleration.From<C>() * lengthScale, color, thickness);
        VisualizationUtils.DrawAngularVelocityArrow(drawing, message.angular_velocity.From<C>(), Vector3.zero, color, sphereRadius, thickness);
    }

    public override Action CreateGUI(ImuMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.orientation.GUI("Orientation");
            message.angular_velocity.GUI("Angular velocity");
            message.linear_acceleration.GUI("Linear acceleration");
            VisualizationUtils.GUIGrid(message.orientation_covariance, 3, "Orientation covariance", ref m_ViewOrientation);
            VisualizationUtils.GUIGrid(message.angular_velocity_covariance, 3, "Angular velocity covariance", ref m_ViewAngular);
            VisualizationUtils.GUIGrid(message.linear_acceleration_covariance, 3, "Linear acceleration covariance", ref m_ViewAccel);
        };
    }
}
