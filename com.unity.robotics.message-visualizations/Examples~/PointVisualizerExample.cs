using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using MPoint = RosMessageTypes.Geometry.Point;

public class PointVisualizerExample : BasicVisualizer<MPoint>
{
	public float size = 0.1f; // this will appear as a configurable parameter in the Unity editor.
	
	public override void Draw(MPoint msg, MessageMetadata meta, Color color, string label,
			DebugDraw.Drawing drawing)
	{
		// The MessageVisualizations class provides convenient exension methods
		// for drawing many common message types.
		msg.Draw<FLU>(drawing, color, size);
		// Or you can directly use the drawing functions provided by the Drawing class
		drawing.DrawLabel(label, message.From<FLU>(), color, size);
	}

	public override System.Action CreateGUI(MPoint msg, MessageMetadata meta, DebugDraw.Drawing drawing)
	{
		// if you want to do any preprocessing or declare any state variables for the GUI to use,
		// you can do that here.
		string text = $"[{message.x}, {message.y}, {message.z}]";
		
		return () =>
		{
			// within the body of this function, it acts like a normal Unity OnGUI function.
			GUILayout.Label(text);
		};
	}
}
