using System;
using RosMessageTypes.Trajectory;
using Unity.Robotics.Visualizations;
using UnityEngine;
#if URDF_IMPORTER
using Unity.Robotics.UrdfImporter;
#endif

public class JointTrajectoryDefaultVisualizer : DrawingVisualizer<JointTrajectoryMsg>
{
#if URDF_IMPORTER
    [SerializeField]
    UrdfRobot m_UrdfRobot;
    RobotVisualization m_RobotData;
#endif
    [SerializeField]
    float m_PathThickness = 0.01f;
    [SerializeField]
    Color m_Color;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;


    public override void Start()
    {
        base.Start();
#if URDF_IMPORTER
        if (m_UrdfRobot != null)
        {
            m_RobotData = new RobotVisualization(m_UrdfRobot);
        }
#endif
    }

    public override void Draw(Drawing3d drawing, JointTrajectoryMsg message, MessageMetadata meta)
    {
#if URDF_IMPORTER
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        m_RobotData.DrawJointPaths(drawing, message, SelectColor(m_Color, meta), m_PathThickness);
#endif
    }

#if !URDF_IMPORTER
    public override Action CreateGUI(JointTrajectoryMsg message, MessageMetadata meta)
    {
        return () =>
        {
            GUILayout.Label("To use the default visualizer for JointTrajectory, please add urdf-importer to your project.");
            GUILayout.Label("https://github.com/Unity-Technologies/URDF-Importer");
        };
    }
#endif
}
