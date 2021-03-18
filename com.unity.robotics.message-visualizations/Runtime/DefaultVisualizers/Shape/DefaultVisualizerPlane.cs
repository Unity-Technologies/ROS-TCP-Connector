using RosMessageTypes.Shape;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPlane : BasicVisualizer<MPlane>
{
    public override void Draw(BasicDrawing drawing, MPlane message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color);
    }

    public override Action CreateGUI(MPlane message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        GUILayout.Label($"[{ message.coef[0]}, {message.coef[1]}, {message.coef[2]}, {message.coef[3]}]");
    };
}
