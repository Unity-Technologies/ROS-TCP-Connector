﻿using RosMessageTypes.Geometry;
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
    [SerializeField]
    [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
    bool m_DrawUnityAxes;

    public override void Draw(DebugDraw.Drawing drawing, MPoseWithCovariance message, MessageMetadata meta, Color color, string label)
    {
        message.pose.Draw<FLU>(drawing, m_Size, m_DrawUnityAxes);
    }

    public override Action CreateGUI(MPoseWithCovariance message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
    {
        message.pose.GUI();
        MessageVisualizations.GUIGrid(message.covariance, 6);
    };
}