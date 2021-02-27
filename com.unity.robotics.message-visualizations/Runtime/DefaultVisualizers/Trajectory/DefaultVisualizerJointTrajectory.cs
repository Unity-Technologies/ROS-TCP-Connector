using RosMessageTypes.Trajectory;
using RosSharp.Urdf;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJointTrajectory : BasicVisualizer<MJointTrajectory>
{
    [SerializeField]
    UrdfRobot m_UrdfRobot;
    [SerializeField]
    float m_PathThickness;
    RobotVisualization m_RobotData;

    public override void Start()
    {
        base.Start();
        m_RobotData = new RobotVisualization(m_UrdfRobot);
    }

    public override void Draw(MJointTrajectory message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
    {
        m_RobotData.DrawJointPaths(drawing, message, color, m_PathThickness);
    }

    public override Action CreateGUI(MJointTrajectory message, MessageMetadata meta, DebugDraw.Drawing drawing)
    {
        return base.CreateGUI(message, meta, drawing);
    }
}
