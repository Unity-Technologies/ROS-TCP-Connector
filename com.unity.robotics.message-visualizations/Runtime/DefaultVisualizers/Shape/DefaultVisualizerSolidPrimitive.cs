using RosMessageTypes.Shape;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerSolidPrimitive : BasicVisualizer<MSolidPrimitive>
{
    [SerializeField]
    public GameObject m_Origin;

    public override void Draw(BasicDrawing drawing, MSolidPrimitive message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color, m_Origin);
    }

    public override Action CreateGUI(MSolidPrimitive message, MessageMetadata meta, BasicDrawing drawing)
    {
        return base.CreateGUI(message, meta, drawing);
    }
}
