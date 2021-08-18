using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class MultiDOFJointStateDefaultVisualizer : DrawingVisualizer<MultiDOFJointStateMsg>
{
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

    public override void Draw(BasicDrawing drawing, MultiDOFJointStateMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        m_RobotData.DrawGhost(drawing, message, SelectColor(m_Color, meta));
    }

    public override Action CreateGUI(MultiDOFJointStateMsg message, MessageMetadata meta)
    {
        var tr = message.transforms.Length > 0;
        var tw = message.twist.Length > 0;
        var wr = message.wrench.Length > 0;

        return () =>
        {
            message.header.GUI();
            for (var i = 0; i < message.joint_names.Length; i++)
            {
                GUILayout.Label($"{message.joint_names[i]}");
                if (tr) message.transforms[i].GUI();
                if (tw) message.twist[i].GUI();
                if (wr) message.wrench[i].GUI();
            }
        };
    }
}
