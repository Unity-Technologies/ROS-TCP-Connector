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
        MessageVisualizations.Draw<FLU>(drawing, message.pose.pose, m_Size);
    }

    public override Action CreateGUI(MPoseWithCovarianceStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
    {
        MessageVisualizations.GUI(message.header);
        MessageVisualizations.GUI(message.pose.pose);
        MessageVisualizations.GUIGrid(message.pose.covariance, 6);
    };
}
