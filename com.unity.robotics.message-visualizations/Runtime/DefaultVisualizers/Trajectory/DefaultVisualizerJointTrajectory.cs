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
    public UrdfRobot urdfRobot;
    public float pathThickness;
    RobotVisualization robotData;

    public override void Start()
    {
        base.Start();
        robotData = new RobotVisualization(urdfRobot);
    }

    public override void Draw(MJointTrajectory message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
    {
        RobotVisualization.JointPlacement[][] jointPlacements = robotData.GetJointPlacements(message);
        for (int Idx = 0; Idx < message.joint_names.Length; ++Idx)
        {
            drawing.DrawLines(color, pathThickness, jointPlacements.Select(p=>p[Idx].position).ToArray());
        }
    }

    public override Action CreateGUI(MJointTrajectory message, MessageMetadata meta, DebugDraw.Drawing drawing)
    {
        return base.CreateGUI(message, meta, drawing);
    }
}
