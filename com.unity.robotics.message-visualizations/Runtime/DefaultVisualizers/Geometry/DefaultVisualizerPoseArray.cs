using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPoseArray : DrawingVisualFactory<PoseArrayMsg>
{
    [SerializeField]
    float m_Size = 0.1f;
    [SerializeField]
    [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
    bool m_DrawUnityAxes;

    public override void Draw(BasicDrawing drawing, PoseArrayMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, m_Size, m_DrawUnityAxes);
    }

    public static void Draw<C>(PoseArrayMsg message, BasicDrawing drawing, float size = 0.1f, bool drawUnityAxes = false) where C : ICoordinateSpace, new()
    {
        foreach (PoseMsg pose in message.poses)
        {
            DefaultVisualizerPose.Draw<C>(pose, drawing, size, drawUnityAxes);
        }
    }

    public override Action CreateGUI(PoseArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
