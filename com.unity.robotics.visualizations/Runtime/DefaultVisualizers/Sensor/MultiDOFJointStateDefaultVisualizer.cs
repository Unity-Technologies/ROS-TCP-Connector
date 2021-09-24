using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using UnityEngine;
#if URDF_IMPORTER
using Unity.Robotics.UrdfImporter;
#endif

public class MultiDOFJointStateDefaultVisualizer : DrawingVisualizer<MultiDOFJointStateMsg>
{
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

    public override void Draw(Drawing3d drawing, MultiDOFJointStateMsg message, MessageMetadata meta)
    {
#if URDF_IMPORTER
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        m_RobotData.DrawGhost(drawing, message, SelectColor(m_Color, meta));
#endif
    }

    public override Action CreateGUI(MultiDOFJointStateMsg message, MessageMetadata meta)
    {
#if URDF_IMPORTER
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
#else
        return () =>
        {
            GUILayout.Label("To use the default visualizer for MultiDOFJointState, please add urdf-importer to your project.");
            GUILayout.Label("https://github.com/Unity-Technologies/URDF-Importer");
        };
#endif
    }
}
