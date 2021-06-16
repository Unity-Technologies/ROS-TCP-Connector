using RosMessageTypes.Nav;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerGridCells : VisualFactory<MGridCells>
{
    [SerializeField]
    float m_Radius = 0.1f;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MGridCells message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Radius);
    }

    public override Action CreateGUI(MGridCells message, MessageMetadata meta) => () =>
    {
        //message.GUI();
    };
}
