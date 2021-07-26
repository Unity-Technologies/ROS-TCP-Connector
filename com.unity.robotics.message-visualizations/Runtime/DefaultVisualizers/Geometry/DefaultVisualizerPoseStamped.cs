using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPoseStamped : DrawingStampedVisualFactory<PoseStampedMsg>
{
    [SerializeField]
    float m_Size = 0.1f;
    [SerializeField]
    [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
    bool m_DrawUnityAxes;

    public override void Draw(BasicDrawing drawing, PoseStampedMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        message.pose.Draw<FLU>(drawing, m_Size, m_DrawUnityAxes);
    }

    public override Action CreateGUI(PoseStampedMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.pose.GUI();
        };
    }
}
