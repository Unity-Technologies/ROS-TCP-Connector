using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPose : BasicVisualizer<MPose>
{
    [SerializeField]
    float m_Size = 0.1f;

    public override void Draw(DebugDraw.Drawing drawing, MPose message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, m_Size);
    }

    public override Action CreateGUI(MPose message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
    {
        message.GUI();
    };
}
