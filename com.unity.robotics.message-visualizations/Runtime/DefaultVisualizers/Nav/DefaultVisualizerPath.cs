using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPath : DrawingVisualFactory<PathMsg>
{
    [SerializeField]
    float m_Thickness;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, PathMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Thickness);
    }

    public override Action CreateGUI(PathMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            foreach (PoseStampedMsg pose in message.poses)
            {
                pose.header.GUI();
                pose.pose.GUI();
            }
        };
    }
}
