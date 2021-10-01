using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PoseWithCovarianceStampedDefaultVisualizer : DrawingVisualizer<PoseWithCovarianceStampedMsg>
{
    [SerializeField]
    float m_Size = 0.1f;
    [SerializeField]
    [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
    bool m_DrawUnityAxes;
    bool m_ViewCovariance;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;

    public override void Draw(Drawing3d drawing, PoseWithCovarianceStampedMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        PoseDefaultVisualizer.Draw<FLU>(message.pose.pose, drawing, m_Size, m_DrawUnityAxes);
    }

    public override Action CreateGUI(PoseWithCovarianceStampedMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.pose.pose.GUI();
            VisualizationUtils.GUIGrid(message.pose.covariance, 6, ref m_ViewCovariance);
        };
    }
}
