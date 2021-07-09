using System;
using RosMessageTypes.Trajectory;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJointTrajectory : DrawingVisualFactory<JointTrajectoryMsg>
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
        if (m_UrdfRobot != null)
        {
            m_RobotData = new RobotVisualization(m_UrdfRobot);
        }
    }

    public override void Draw(BasicDrawing drawing, JointTrajectoryMsg message, MessageMetadata meta)
    {
        m_RobotData.DrawJointPaths(drawing, message, SelectColor(m_Color, meta), m_PathThickness);
    }
}
