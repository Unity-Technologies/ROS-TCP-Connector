using RosMessageTypes.Nav;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerGridCells : BasicVisualizer<MGridCells>
{
    [SerializeField]
    float m_Radius = 0.1f;

    public override void Draw(BasicDrawing drawing, MGridCells message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color, m_Radius);
    }

    public override Action CreateGUI(MGridCells message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        //message.GUI();
    };
}
