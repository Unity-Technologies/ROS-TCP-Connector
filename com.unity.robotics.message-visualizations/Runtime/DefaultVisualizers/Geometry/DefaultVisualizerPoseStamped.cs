using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPoseStamped : BasicVisualizer<MPoseStamped>
{
    [SerializeField]
    float m_Size = 0.1f;

    public override void Draw(DebugDraw.Drawing drawing, MPoseStamped message, MessageMetadata meta, Color color, string label)
    {
        MessageVisualizations.Draw<FLU>(drawing, message.pose, m_Size);
    }

    public override Action CreateGUI(MPoseStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
    {
        MessageVisualizations.GUI(message.header);
        MessageVisualizations.GUI(message.pose);
    };
}
