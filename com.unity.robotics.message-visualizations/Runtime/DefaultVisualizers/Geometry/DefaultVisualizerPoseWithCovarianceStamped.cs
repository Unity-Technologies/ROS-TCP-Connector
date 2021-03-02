using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPoseWithCovarianceStamped : BasicVisualizer<MPoseWithCovarianceStamped>
{
    [SerializeField]
    float m_Size = 0.1f;

    public override void Draw(DebugDraw.Drawing drawing, MPoseWithCovarianceStamped message, MessageMetadata meta, Color color, string label)
    {
        message.pose.pose.Draw<FLU>(drawing, m_Size);
    }

    public override Action CreateGUI(MPoseWithCovarianceStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
    {
        message.header.GUI();
        message.pose.pose.GUI();
        MessageVisualizations.GUIGrid(message.pose.covariance, 6);
    };
}
