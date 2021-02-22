using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using MPolygon = RosMessageTypes.Geometry.Polygon;
using MPoint32 = RosMessageTypes.Geometry.Point32;

public class DefaultVisualizerPolygon : VisualizerConfigWithDrawing<MPolygon>
{
    public float thickness = 0.1f;

    public override void Draw(MPolygon msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
    {
        MessageVisualizations.Draw<FLU>(drawing, msg, color, thickness);
    }

    public override System.Action CreateGUI(MPolygon msg, MessageMetadata meta, object drawing) => () =>
    {
        foreach(MPoint32 p in msg.points)
            MessageVisualizations.GUI(label, p);
    };
}
