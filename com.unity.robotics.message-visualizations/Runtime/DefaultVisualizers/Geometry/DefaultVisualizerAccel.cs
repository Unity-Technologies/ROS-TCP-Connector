using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerAccel : BasicVisualizer<MAccel>
{
    public float m_Thickness = 0.01f;
    public float m_LengthScale = 1.0f;
    public float m_SphereRadius = 1.0f;
    public GameObject m_Origin;

    public override void Draw(BasicDrawing drawing, MAccel message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color, m_Origin, m_LengthScale, m_SphereRadius, m_Thickness );
    }

    public override Action CreateGUI(MAccel message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI();
    };
}
