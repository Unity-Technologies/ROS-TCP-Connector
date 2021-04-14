using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPointCloud : BasicVisualizer<MPointCloud>
{
    public override void Draw(BasicDrawing drawing, MPointCloud message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color);
    }

    public override Action CreateGUI(MPointCloud message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.GUI();
    };
}
