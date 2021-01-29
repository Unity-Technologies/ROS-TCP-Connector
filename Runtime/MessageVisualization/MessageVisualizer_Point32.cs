using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using ROSGeometry;

[VisualizeMessage(typeof(Point))]
public class MessageVisualizer_Point32 : MessageVisualizerDrawing<Point>
{
    public override void DrawGUI(DebugDraw.Drawing drawing, Point message)
    {
        GUILayout.Label("Hello!", HUDPanel.boldStyle);
        GUILayout.Label("X: " + message.x + " Y: " + message.y + " Z: " + message.z + " bla bla bla bla bla bla bla bla bla bla bla bla bla bla");
    }

    public override void Draw(DebugDraw.Drawing drawing, Point message)
    {
        MessageVisualizations<FLU>.DrawPoint(drawing, message, Color.green, "Hello!");
    }
}
