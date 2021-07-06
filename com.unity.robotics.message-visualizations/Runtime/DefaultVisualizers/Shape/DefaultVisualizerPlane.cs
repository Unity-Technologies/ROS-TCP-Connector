using System;
using RosMessageTypes.Shape;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPlane : DrawingVisualFactory<PlaneMsg>
{
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, PlaneMsg message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta));
    }

    public override Action CreateGUI(PlaneMsg message, MessageMetadata meta)
    {
        return () =>
        {
            GUILayout.Label($"[{message.coef[0]}, {message.coef[1]}, {message.coef[2]}, {message.coef[3]}]");
        };
    }
}
