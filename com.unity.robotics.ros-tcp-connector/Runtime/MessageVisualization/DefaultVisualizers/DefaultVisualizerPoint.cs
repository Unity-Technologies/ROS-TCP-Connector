using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using MPoint = RosMessageTypes.Geometry.Point;

public class DefaultVisualizerPoint : BasicVisualizerWithPriority<MPoint>
{
    public float size = 0.01f;

    public override void Draw(MPoint msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
    {
        MessageVisualizations.Draw<FLU>(drawing, msg, color, label, size);
    }

    public override System.Action CreateGUI(MPoint msg, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
    {
        MessageVisualizations.GUI(label, msg);
    };
}
