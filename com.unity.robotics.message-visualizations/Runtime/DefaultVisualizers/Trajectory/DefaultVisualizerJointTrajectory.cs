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
    float m_PathThickness = 0.01f;
    RobotVisualization m_RobotData;

    public override void Start()
    {
        base.Start();
        m_RobotData = new RobotVisualization(m_UrdfRobot);
    }

    public override void Draw(BasicDrawing drawing, MJointTrajectory message, MessageMetadata meta, Color color, string label)
    {
        m_RobotData.DrawJointPaths(drawing, message, color, m_PathThickness);
    }

    public override Action CreateGUI(MJointTrajectory message, MessageMetadata meta, BasicDrawing drawing)
    {
        return base.CreateGUI(message, meta, drawing);
    }
}
