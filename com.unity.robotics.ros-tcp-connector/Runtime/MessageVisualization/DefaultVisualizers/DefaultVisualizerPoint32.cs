using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using MPoint32 = RosMessageTypes.Geometry.Point32;

public class DefaultVisualizerPoint32 : VisualizerConfigWithDrawing<MPoint32>
{
    public float size = 0.01f;

    public override void Draw(MPoint32 msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
    {
        MessageVisualizations.Draw<FLU>(drawing, msg, color, label, size);
    }

    public override System.Action CreateGUI(MPoint32 msg, MessageMetadata meta, object drawing) => () =>
    {
        MessageVisualizations.GUI(label, msg);
    };
}
