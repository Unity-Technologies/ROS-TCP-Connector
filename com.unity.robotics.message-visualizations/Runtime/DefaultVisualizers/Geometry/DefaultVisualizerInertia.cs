using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerInertia : VisualFactory<MInertia>
{
    public GameObject m_Origin;
    public float m_Radius;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MInertia message, MessageMetadata meta)
    {
        message.com.Draw<FLU>(drawing, m_Origin, SelectColor(m_Color, meta), "Center of mass", m_Radius);
    }

    public override Action CreateGUI(MInertia message, MessageMetadata meta) => () =>
    {
        message.GUI();
    };
}
