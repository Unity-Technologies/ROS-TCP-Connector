using RosMessageTypes.Shape;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerSolidPrimitive : VisualFactory<MSolidPrimitive>
{
    [SerializeField]
    GameObject m_Origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MSolidPrimitive message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Origin);
    }

    public override Action CreateGUI(MSolidPrimitive message, MessageMetadata meta) => () =>
    {
        message.GUI();
    };
}
