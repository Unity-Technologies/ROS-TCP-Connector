using System;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerOccupancyGrid : DrawingVisualFactory<OccupancyGridMsg>
{
    public override void Draw(BasicDrawing drawing, OccupancyGridMsg message, MessageMetadata meta)
    {
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
