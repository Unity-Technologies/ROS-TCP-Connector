using System;
using RosMessageTypes.Shape;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMesh : DrawingVisualFactory<MeshMsg>
{
    [SerializeField]
    GameObject m_Origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MeshMsg message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Origin);
    }

    public override Action CreateGUI(MeshMsg message, MessageMetadata meta)
    {
        var showTriangles = false;
        return () =>
        {
            showTriangles = GUILayout.Toggle(showTriangles, $"Show {message.vertices.Length} vertices");
            if (showTriangles)
                message.GUI();
        };
    }
}
