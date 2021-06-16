using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerAccelWithCovarianceStamped : VisualFactory<MAccelWithCovarianceStamped>
{
    public float m_Thickness = 0.01f;
    public float m_LengthScale = 1.0f;
    public float m_SphereRadius = 1.0f;
    public GameObject m_Origin;
    [SerializeField]
    Color m_Color;
    bool m_ViewCovariance;

    public override void Draw(BasicDrawing drawing, MAccelWithCovarianceStamped message, MessageMetadata meta)
    {
        message.accel.accel.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Origin, m_LengthScale, m_SphereRadius, m_Thickness );
    }

    public override Action CreateGUI(MAccelWithCovarianceStamped message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        message.accel.accel.GUI();
        MessageVisualizations.GUIGrid(message.accel.covariance, 6, ref m_ViewCovariance);
    };
}
