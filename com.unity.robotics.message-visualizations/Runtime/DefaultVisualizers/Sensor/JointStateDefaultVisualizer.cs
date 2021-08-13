using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class JointStateDefaultVisualizer : DrawingVisualizer<JointStateMsg>
{
    [SerializeField]
    bool m_ShowEffort;
    [SerializeField]
    UrdfRobot m_UrdfRobot;
    [SerializeField]
    Color m_Color;
    [SerializeField]
    RobotVisualization m_RobotData;

    public override void Start()
    {
        base.Start();
        if (m_UrdfRobot != null)
            m_RobotData = new RobotVisualization(m_UrdfRobot);
    }

    public override void Draw(BasicDrawing drawing, JointStateMsg message, MessageMetadata meta)
    {
        var color = SelectColor(m_Color, meta);
        m_RobotData.DrawGhost(drawing, message, color);
        if (m_ShowEffort)
            m_RobotData.DrawEffort(drawing, message, color);
    }

    public override Action CreateGUI(JointStateMsg message, MessageMetadata meta)
    {
        var pos = message.position.Length > 0;
        var vel = message.velocity.Length > 0;
        var eff = message.effort.Length > 0;
        var s = "";

        for (var i = 0; i < message.name.Length; i++)
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
