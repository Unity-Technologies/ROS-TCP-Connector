using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPoseWithCovariance : BasicVisualizer<MPoseWithCovariance>
{
    [SerializeField]
    float m_Size = 0.1f;

    public override void Draw(DebugDraw.Drawing drawing, MPoseWithCovariance message, MessageMetadata meta, Color color, string label)
    {
        message.pose.Draw<FLU>(drawing, m_Size);
    }

    public override Action CreateGUI(MPoseWithCovariance message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
    {
        message.pose.GUI();
        MessageVisualizations.GUIGrid(message.covariance, 6);
    };
}
