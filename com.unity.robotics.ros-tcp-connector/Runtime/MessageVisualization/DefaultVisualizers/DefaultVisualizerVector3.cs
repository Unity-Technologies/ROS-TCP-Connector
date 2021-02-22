using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using MVector3 = RosMessageTypes.Geometry.Vector3;

public class DefaultVisualizerVector3 : VisualizerConfigWithDrawing<MVector3>
{
    public float size = 0.01f;

    public override void Draw(MVector3 msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
    {
        MessageVisualizations.Draw<FLU>(drawing, msg, color, label, size);
    }

    public override System.Action CreateGUI(MVector3 msg, MessageMetadata meta, object drawing) => () =>
    {
        MessageVisualizations.GUI(label, msg);
    };
}
