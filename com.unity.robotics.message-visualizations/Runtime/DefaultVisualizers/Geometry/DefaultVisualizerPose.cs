using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPose : DrawingVisualFactory<PoseMsg>
{
    [SerializeField]
    float m_Size = 0.1f;
    [SerializeField]
    [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
    bool m_DrawUnityAxes;

    public override void Draw(BasicDrawing drawing, PoseMsg message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, m_Size, m_DrawUnityAxes);
    }

    public override Action CreateGUI(PoseMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
