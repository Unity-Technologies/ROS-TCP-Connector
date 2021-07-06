using System;
using RosMessageTypes.Shape;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerSolidPrimitive : DrawingVisualFactory<SolidPrimitiveMsg>
{
    [SerializeField]
    GameObject m_Origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, SolidPrimitiveMsg message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Origin);
    }

    public override Action CreateGUI(SolidPrimitiveMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
