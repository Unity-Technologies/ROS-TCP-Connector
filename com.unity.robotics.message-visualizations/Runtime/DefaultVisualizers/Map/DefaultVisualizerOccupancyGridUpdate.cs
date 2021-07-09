using System;
using RosMessageTypes.Map;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerOccupancyGridUpdate : DrawingVisualFactory<OccupancyGridUpdateMsg>
{
    public override void Draw(BasicDrawing drawing, OccupancyGridUpdateMsg message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing);
    }

    public override Action CreateGUI(OccupancyGridUpdateMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"(x, y): ({message.x}, {message.y})");
            GUILayout.Label($"Width x height: {message.width} x {message.height}");
        };
    }
}
