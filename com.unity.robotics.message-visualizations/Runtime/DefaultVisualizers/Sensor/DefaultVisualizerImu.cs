using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerImu : BasicVisualizer<MImu>
{
    public Color m_Color;
    public override void Draw(BasicDrawing drawing, MImu message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta));
    }

    public override Action CreateGUI(MImu message, MessageMetadata meta, BasicDrawing drawing) => () =>
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
