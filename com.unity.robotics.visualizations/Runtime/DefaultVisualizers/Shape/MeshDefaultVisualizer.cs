using System;
using System.Linq;
using RosMessageTypes.Shape;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class MeshDefaultVisualizer : DrawingVisualizer<MeshMsg>
{
    [SerializeField]
    GameObject m_Origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(Drawing3d drawing, MeshMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), m_Origin);
    }

    public static void Draw<C>(MeshMsg message, Drawing3d drawing, Color color, GameObject origin = null) where C : ICoordinateSpace, new()
    {
        Mesh mesh = new Mesh();
        mesh.vertices = message.vertices.Select(v => v.From<C>()).ToArray();
        mesh.triangles = message.triangles.SelectMany(tri => tri.vertex_indices.Select(i => (int)i)).ToArray();
        if (origin != null)
            drawing.DrawMesh(mesh, origin.transform, color);
        else
            drawing.DrawMesh(mesh, Vector3.zero, Quaternion.identity, Vector3.one, color);
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
