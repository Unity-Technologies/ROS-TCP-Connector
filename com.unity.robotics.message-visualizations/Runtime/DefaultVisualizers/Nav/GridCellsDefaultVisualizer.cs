using System;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class GridCellsDefaultVisualizer : DrawingVisualizer<GridCellsMsg>
{
    [SerializeField]
    float m_Radius = 0.1f;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, GridCellsMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), m_Radius);
    }

    public static void Draw<C>(GridCellsMsg message, BasicDrawing drawing, Color color, float radius = 0.01f)
        where C : ICoordinateSpace, new()
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        MessageVisualizationUtils.DrawPointCloud<C>(message.cells, drawing, color, radius);
    }

    public override Action CreateGUI(GridCellsMsg message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Cell width x height: {message.cell_width} x {message.cell_height}");
    };
}
