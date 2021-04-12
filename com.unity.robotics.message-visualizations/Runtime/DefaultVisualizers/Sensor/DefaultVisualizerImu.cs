using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerImu : BasicVisualizer<MImu>
{
    public override void Draw(BasicDrawing drawing, MImu message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color);
    }

    public override Action CreateGUI(MImu message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.GUI();
        MessageVisualizations.GUIGrid(message.orientation_covariance, 3);
        MessageVisualizations.GUIGrid(message.angular_velocity_covariance, 3);
        MessageVisualizations.GUIGrid(message.linear_acceleration_covariance, 3);
    };
}
