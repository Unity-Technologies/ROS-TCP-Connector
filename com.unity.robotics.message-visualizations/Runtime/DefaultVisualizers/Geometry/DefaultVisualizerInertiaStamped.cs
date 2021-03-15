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

    public override void Draw(BasicDrawing drawing, MInertiaStamped message, MessageMetadata meta, Color color, string label)
    {
        message.inertia.com.Draw<FLU>(drawing, m_Origin, color, "Center of mass", m_Radius);
    }

    public override Action CreateGUI(MInertiaStamped message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.inertia.GUI();
    };
}
