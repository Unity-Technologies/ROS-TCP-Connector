using RosMessageTypes.Trajectory;
using RosSharp.Urdf;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJointTrajectory : BasicVisualFactory<MJointTrajectory>
{
    [SerializeField]
    UrdfRobot m_UrdfRobot;
    [SerializeField]
    float m_PathThickness = 0.01f;
    [SerializeField]
    Color m_Color;

    RobotVisualization m_RobotData;

    public override void Start()
    {
        base.Start();
        if(m_UrdfRobot != null)
            m_RobotData = new RobotVisualization(m_UrdfRobot);
    }

    public override void Draw(BasicDrawing drawing, MJointTrajectory message, MessageMetadata meta)
    {
        m_RobotData.DrawJointPaths(drawing, message, SelectColor(m_Color, meta), m_PathThickness);
    }
}
