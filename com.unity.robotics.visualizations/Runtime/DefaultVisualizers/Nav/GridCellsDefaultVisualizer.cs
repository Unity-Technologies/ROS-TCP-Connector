using System;
using RosMessageTypes.Nav;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class GridCellsDefaultVisualizer : DrawingVisualizer<GridCellsMsg>
{
    [SerializeField]
    float m_Radius = 0.1f;
    [SerializeField]
    Color m_Color;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;
    public override void Draw(Drawing3d drawing, GridCellsMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), m_Radius);
    }

    public static void Draw<C>(GridCellsMsg message, Drawing3d drawing, Color color, float radius = 0.01f)
        where C : ICoordinateSpace, new()
    {
        VisualizationUtils.DrawPointCloud<C>(message.cells, drawing, color, radius);
    }

    public override Action CreateGUI(GridCellsMsg message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Cell width x height: {message.cell_width} x {message.cell_height}");
    };
}
