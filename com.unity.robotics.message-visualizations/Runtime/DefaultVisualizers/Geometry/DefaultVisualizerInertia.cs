using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerInertia : BasicVisualizer<MInertia>
{
    public GameObject m_Origin;
    public float m_Radius;

    public override void Draw(DebugDraw.Drawing drawing, MInertia message, MessageMetadata meta, Color color, string label)
    {
        message.com.Draw<FLU>(drawing, m_Origin, color, "Center of mass", m_Radius);
    }

    public override Action CreateGUI(MInertia message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
    {
        message.GUI();
    };
}
