using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerInertia : DrawingVisualFactory<InertiaMsg>
{
    public GameObject m_Origin;
    public float m_Radius;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, InertiaMsg message, MessageMetadata meta)
    {
        message.com.Draw<FLU>(drawing, m_Origin, SelectColor(m_Color, meta), "Center of mass", m_Radius);
    }

    public override Action CreateGUI(InertiaMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
