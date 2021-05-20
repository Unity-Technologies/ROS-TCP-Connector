using RosMessageTypes.Sensor;
using RosSharp.Urdf;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerMultiDOFJointState : BasicVisualizer<MMultiDOFJointState>
{
    [SerializeField]
    UrdfRobot m_UrdfRobot;
    [SerializeField]
    RobotVisualization m_RobotData;
    [SerializeField]
    Color m_Color;

    public override void Start()
    {
        base.Start();
        if(m_UrdfRobot != null)
            m_RobotData = new RobotVisualization(m_UrdfRobot);
    }

    public override void Draw(BasicDrawing drawing, MMultiDOFJointState message, MessageMetadata meta)
    {
        m_RobotData.DrawGhost(drawing, message, SelectColor(m_Color, meta));
    }

    public override Action CreateGUI(MMultiDOFJointState message, MessageMetadata meta, BasicDrawing drawing)
    {
        bool tr = message.transforms.Length > 0;
        bool tw = message.twist.Length > 0;
        bool wr = message.wrench.Length > 0;

        return () =>
        {
            message.header.GUI();
            for (int i = 0; i < message.joint_names.Length; i++)
            {
                GUILayout.Label($"{message.joint_names[i]}");
                if (tr) message.transforms[i].GUI();
                if (tw) message.twist[i].GUI();
                if (wr) message.wrench[i].GUI();
            }
        };
    }
}
