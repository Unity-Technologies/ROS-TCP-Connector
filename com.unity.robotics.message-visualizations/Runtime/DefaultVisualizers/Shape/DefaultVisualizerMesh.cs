using RosMessageTypes.Shape;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMesh : VisualFactory<MMesh>
{
    [SerializeField]
    GameObject m_Origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MMesh message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Origin);
    }

    public override Action CreateGUI(MMesh message, MessageMetadata meta)
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
