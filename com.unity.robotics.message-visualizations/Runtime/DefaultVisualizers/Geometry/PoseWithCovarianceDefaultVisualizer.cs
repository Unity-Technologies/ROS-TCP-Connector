using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PoseWithCovarianceDefaultVisualizer : DrawingVisualizer<PoseWithCovarianceMsg>
{
    [SerializeField]
    float m_Size = 0.1f;
    [SerializeField]
    [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
    bool m_DrawUnityAxes;
    bool m_ViewCovariance;

    public override void Draw(BasicDrawing drawing, PoseWithCovarianceMsg message, MessageMetadata meta)
    {
        PoseDefaultVisualizer.Draw<FLU>(message.pose, drawing, m_Size, m_DrawUnityAxes);
    }

    public override Action CreateGUI(PoseWithCovarianceMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.pose.GUI();
            MessageVisualizationUtils.GUIGrid(message.covariance, 6, ref m_ViewCovariance);
        };
    }
}
