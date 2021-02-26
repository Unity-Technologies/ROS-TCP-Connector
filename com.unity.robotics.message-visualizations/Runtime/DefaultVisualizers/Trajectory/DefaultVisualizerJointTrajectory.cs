using RosMessageTypes.Trajectory;
using RosSharp.Urdf;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJointTrajectory : BasicVisualizer<MJointTrajectory>
{
    public UrdfRobot urdfRobot;
    RobotVisualizationData robotData;

    public override void Start()
    {
        base.Start();
        robotData = new RobotVisualizationData(urdfRobot);
    }

    public override void Draw(MJointTrajectory message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
    {
        MessageVisualizations.Draw(drawing, message.joint_names, message.points, color);
    }

    public override Action CreateGUI(MJointTrajectory message, MessageMetadata meta, DebugDraw.Drawing drawing)
    {
        return base.CreateGUI(message, meta, drawing);
    }
}
