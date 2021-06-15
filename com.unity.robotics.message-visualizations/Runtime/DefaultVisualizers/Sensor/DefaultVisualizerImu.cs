using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerImu : BasicVisualFactory<MImu>
{
    [SerializeField]
    public Color m_Color;
    [SerializeField]
    public float m_LengthScale = 1;
    [SerializeField]
    public float m_SphereRadius = 1;
    [SerializeField]
    public float m_Thickness = 0.01f;

    public override void Draw(BasicDrawing drawing, MImu message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_LengthScale, m_SphereRadius, m_Thickness);
    }

    public override Action CreateGUI(MImu message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        message.orientation.GUI("Orientation");
        message.angular_velocity.GUI("Angular velocity");
        message.linear_acceleration.GUI("Linear acceleration");
        MessageVisualizations.GUIGrid(message.orientation_covariance, 3, "Orientation covariance:");
        MessageVisualizations.GUIGrid(message.angular_velocity_covariance, 3, "Angular velocity covariance:");
        MessageVisualizations.GUIGrid(message.linear_acceleration_covariance, 3, "Linear acceleration covariance:");
    };
}
