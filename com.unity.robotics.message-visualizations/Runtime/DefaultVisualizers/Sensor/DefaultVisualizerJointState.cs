using RosMessageTypes.Sensor;
using RosSharp.Urdf;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJointState : BasicVisualizer<MJointState>
{
    [SerializeField]
    bool m_ShowEffort = false;
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

    public override void Draw(BasicDrawing drawing, MJointState message, MessageMetadata meta)
    {
        Color color = SelectColor(m_Color, meta);
        m_RobotData.DrawGhost(drawing, message, color);
        if (m_ShowEffort)
        {
            m_RobotData.DrawEffort(drawing, message, color);
        }
    }

    public override Action CreateGUI(MJointState message, MessageMetadata meta, BasicDrawing drawing) 
    {
        bool pos = message.position.Length > 0;
        bool vel = message.velocity.Length > 0;
        bool eff = message.effort.Length > 0;
        string s = "";

        for (int i = 0; i < message.name.Length; i++)
        {
            if (s.Length > 0) s += "\n";
            s += $"{message.name[i]}:";
            if (pos) s += $"\nPosition: {message.position[i]}";
            if (vel) s += $"\nVelocity: {message.velocity[i]}";
            if (eff) s += $"\nEffort: {message.effort[i]}";
        }

        return () => 
        {
            message.header.GUI();
            GUILayout.Label(s);
        };
    }
}
