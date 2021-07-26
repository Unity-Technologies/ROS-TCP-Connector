using System;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerGridCells : DrawingStampedVisualFactory<GridCellsMsg>
{
    [SerializeField]
    float m_Radius = 0.1f;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, GridCellsMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Radius);
    }

    public override Action CreateGUI(GridCellsMsg message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Cell width x height: {message.cell_width} x {message.cell_height}");
    };
}
