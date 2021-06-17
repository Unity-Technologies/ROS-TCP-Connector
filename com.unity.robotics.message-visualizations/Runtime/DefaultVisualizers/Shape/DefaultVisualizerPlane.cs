using RosMessageTypes.Shape;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPlane : DrawingVisualFactory<MPlane>
{
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MPlane message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta));
    }

    public override Action CreateGUI(MPlane message, MessageMetadata meta) => () =>
    {
        GUILayout.Label($"[{ message.coef[0]}, {message.coef[1]}, {message.coef[2]}, {message.coef[3]}]");
    };
}
