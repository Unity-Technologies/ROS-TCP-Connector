using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerRange : BasicVisualizer<MRange>
{
    public override void Draw(BasicDrawing drawing, MRange message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color);
    }
        
    public override Action CreateGUI(MRange message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.GUI();
    };
}
