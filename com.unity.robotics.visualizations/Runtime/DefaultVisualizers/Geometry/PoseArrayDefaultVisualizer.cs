using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PoseArrayDefaultVisualizer : DrawingVisualizer<PoseArrayMsg>
{
    [SerializeField]
    float m_Size = 0.1f;
    [SerializeField]
    [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
    bool m_DrawUnityAxes;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;

    public override void Draw(Drawing3d drawing, PoseArrayMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        Draw<FLU>(message, drawing, m_Size, m_DrawUnityAxes);
    }

    public static void Draw<C>(PoseArrayMsg message, Drawing3d drawing, float size = 0.1f, bool drawUnityAxes = false) where C : ICoordinateSpace, new()
    {
        foreach (PoseMsg pose in message.poses)
        {
            PoseDefaultVisualizer.Draw<C>(pose, drawing, size, drawUnityAxes);
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
