using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerAccelWithCovarianceStamped : BasicVisualizer<MAccelWithCovarianceStamped>
{
    public float m_Thickness = 0.01f;
    public float m_LengthScale = 1.0f;
    public float m_SphereRadius = 1.0f;
    public GameObject m_Origin;

    public override void Draw(DebugDraw.Drawing drawing, MAccelWithCovarianceStamped message, MessageMetadata meta, Color color, string label)
    {
        message.accel.accel.Draw<FLU>(drawing, color, m_Origin, m_LengthScale, m_SphereRadius, m_Thickness );
    }

    public override Action CreateGUI(MAccelWithCovarianceStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
    {
        message.header.GUI();
        message.accel.accel.GUI();
        MessageVisualizations.GUIGrid(message.accel.covariance, 6);
    };
}
