using System;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerOccupancyGrid : DrawingVisualFactory<OccupancyGridMsg>
{
    [SerializeField]
    TFTrackingType m_TFTrackingType = TFTrackingType.Exact;
    public TFTrackingType TFTrackingType { get => m_TFTrackingType; set => m_TFTrackingType = value; }

    public override void Draw(BasicDrawing drawing, OccupancyGridMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        message.Draw<FLU>(drawing);
    }

    public override Action CreateGUI(OccupancyGridMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.info.GUI();
        };
    }
}
