using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class InertiaDefaultVisualizer : DrawingVisualizer<InertiaMsg>
{
    public GameObject m_Origin;
    public float m_Radius;
    [SerializeField]
    Color m_Color;

    public override void Draw(Drawing3d drawing, InertiaMsg message, MessageMetadata meta)
    {
        Vector3DefaultVisualizer.Draw<FLU>(message.com, drawing, m_Origin, SelectColor(m_Color, meta), "Center of mass", m_Radius);
    }

    public override Action CreateGUI(InertiaMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
