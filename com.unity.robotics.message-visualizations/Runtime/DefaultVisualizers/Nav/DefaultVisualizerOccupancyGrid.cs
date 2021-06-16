using RosMessageTypes.Nav;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerOccupancyGrid : VisualFactory<MOccupancyGrid>
{
    public override void Draw(BasicDrawing drawing, MOccupancyGrid message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing);
    }

    public override Action CreateGUI(MOccupancyGrid message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        message.info.GUI();
    };
}
