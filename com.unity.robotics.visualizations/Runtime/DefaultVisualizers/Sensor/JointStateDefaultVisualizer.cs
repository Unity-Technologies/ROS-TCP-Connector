using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using UnityEngine;
#if URDF_IMPORTER
using Unity.Robotics.UrdfImporter;
#endif

public class JointStateDefaultVisualizer : DrawingVisualizer<JointStateMsg>
{
    [SerializeField]
    bool m_ShowEffort;
#if URDF_IMPORTER
    [SerializeField]
    UrdfRobot m_UrdfRobot;
    RobotVisualization m_RobotData;
#endif
    [SerializeField]
    Color m_Color;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;

    public override void Start()
    {
        base.Start();
#if URDF_IMPORTER
        if (m_UrdfRobot != null)
            m_RobotData = new RobotVisualization(m_UrdfRobot);
#endif
    }

    public override void Draw(Drawing3d drawing, JointStateMsg message, MessageMetadata meta)
    {
#if URDF_IMPORTER
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        var color = SelectColor(m_Color, meta);
        m_RobotData.DrawGhost(drawing, message, color);
        if (m_ShowEffort)
            m_RobotData.DrawEffort(drawing, message, color);
#endif
    }

    public override Action CreateGUI(JointStateMsg message, MessageMetadata meta)
    {
#if URDF_IMPORTER
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
#else
        return () =>
        {
            GUILayout.Label("To use the default visualizer for JointState, please add urdf-importer to your project.");
            GUILayout.Label("https://github.com/Unity-Technologies/URDF-Importer");
        };
#endif
    }
}
