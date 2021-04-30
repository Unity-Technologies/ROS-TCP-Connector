using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerAccelStamped : BasicVisualizer<MAccelStamped>
{
    public float m_Thickness = 0.01f;
    public float m_LengthScale = 1.0f;
    public float m_SphereRadius = 1.0f;
    public GameObject m_Origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MAccelStamped message, MessageMetadata meta)
    {
        message.accel.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Origin, m_LengthScale, m_SphereRadius, m_Thickness );
    }

    public override Action CreateGUI(MAccelStamped message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.accel.GUI();
    };
}
