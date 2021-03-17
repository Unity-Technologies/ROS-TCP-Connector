using RosMessageTypes.Shape;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMesh : BasicVisualizer<MMesh>
{
    [SerializeField]
    GameObject m_Origin;

    public override void Draw(BasicDrawing drawing, MMesh message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color, m_Origin);
    }

    public override Action CreateGUI(MMesh message, MessageMetadata meta, BasicDrawing drawing)
    {
        bool showTriangles = false;
        return () =>
        {
            showTriangles = GUILayout.Toggle(showTriangles, $"Show {message.vertices.Length} vertices");
            if (showTriangles)
                message.GUI();
        };
    }
}
