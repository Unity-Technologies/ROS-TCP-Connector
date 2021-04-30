using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerInertiaStamped : BasicVisualizer<MInertiaStamped>
{
    public GameObject m_Origin;
    public float m_Radius;
    [SerializeField]
    Color m_Color;
    [SerializeField]
    string m_Label = "Center of mass";

    public override void Draw(BasicDrawing drawing, MInertiaStamped message, MessageMetadata meta)
    {
        message.inertia.com.Draw<FLU>(drawing, m_Origin, SelectColor(m_Color, meta), SelectLabel(m_Label, meta), m_Radius);
    }

    public override Action CreateGUI(MInertiaStamped message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.inertia.GUI();
    };
}
