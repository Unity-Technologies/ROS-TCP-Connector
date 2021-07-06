using System;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerGridCells : DrawingVisualFactory<GridCellsMsg>
{
    [SerializeField]
    float m_Radius = 0.1f;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, GridCellsMsg message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Radius);
    }

    public override Action CreateGUI(GridCellsMsg message, MessageMetadata meta)
    {
        return () =>
        {
            //message.GUI();
        };
    }
}
