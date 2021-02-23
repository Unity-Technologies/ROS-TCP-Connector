# Message Visualization component

In the ROSConnector HUD you can see the last sent (publisher) and received (subscriber) message, and also the request and response of the last completed service. The HUD should appear whenever your ROSConnector starts up. If it doesn't, ensure "Show HUD" is ticked in your ROS Settings window.

You can configure exactly how the HUD displays a message by setting up your own Visualization Suite, and/or creating your own *custom visualizations*.

# Configuring your visualization suite

The simplest way to tune how visualizations appear is to set up your own Visualization Suite. This is a collection of message visualizers, placed in your scene, that control how messages are displayed in that scene. (To share one suite of visualizers between multiple scenes, consider creating a prefab.)

To create a visualization suite, create a new GameObject in your scene, call it "VisualizationSuite" (or whatever you want) and add any number of visualizer components to it. Each visualizer can be set up to apply only to messages on a specific topic - just edit the "topic" field - and can have its own custom color and label.

For example, let's assume you want to call out Point messages on the "important_points" topic, and you'd like them to appear extra large and in red. So maybe you'd add a DefaultVisualizerPoint to your suite, and set it to draw large red points.

![](images~/VisualizationSuiteExample.png)

(Note that in Unity, "size" is in Unity coordinates, so a size of 1 means the points have a radius of 1 meter.)

# Writing a message visualizer using DebugDraw

The simplest way to create a visualizer for your own custom messages is to write a MonoBehaviour script that inherits from VisualizerWithDrawing. Here's a simple example:

	using System.Collections;
	using System.Collections.Generic;
	using Unity.Robotics.MessageVisualizers;
	using Unity.Robotics.ROSTCPConnector.MessageGeneration;
	using Unity.Robotics.ROSTCPConnector.ROSGeometry;
	using UnityEngine;
	using MPoint = RosMessageTypes.Geometry.Point;
	
	public class PointVisualizerExample : VisualizerWithDrawing<MPoint>
	{
		public float size = 0.1f; // this will appear as a configurable parameter in the Unity editor.
		
		public override void Draw(MPoint msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
		{
            drawing.DrawPoint(msg.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size);
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


- When the HUD needs to display the graphics for your message, it will call your Draw() function, providing all the information you need about what drawing to create. The last parameter, `drawing`, is a convenient drawing class you can use to draw text, 3d lines, points, and other geometric shapes. Your Draw function should call whatever functions you want to use on this drawing. The VisualizerWithDrawing class automatically provides a suitable color and label for your drawing, but you don't have to use them if you don't want to.
- When the HUD needs to display the GUI for your message, it will call CreateGUI on your class. CreateGUI should return a function that behaves like a normal Unity OnGUI callback: in other words, it will be invoked once per UI event, for as long as your visualizer is active, and any GUI elements you draw in this function will appear in the HUD. Note this is a runtime GUI, not an editor GUI, so the GUI functions you call should come from GUI or GUILayout, not EditorGUILayout.
- Note, your drawing also gets passed into the CreateGUI function. You can use it to clear and redraw the drawing if you want (for example, maybe your gui provides buttons to turn parts of the drawing on and off, or resize an element).

# Writing a custom message visualizer

If you want to create a visualization more complex than the simple DebugDraw class can support, you can write a MonoBehaviour script that inherits from Visualizer - the base class of VisualizerWithDrawing. Here's an example:

	using System.Collections;
	using System.Collections.Generic;
	using UnityEngine;
	using RosMessageTypes.Geometry;
	using Unity.Robotics.ROSTCPConnector.ROSGeometry;
	using Unity.Robotics.ROSTCPConnector.MessageGeneration;
	
    public class ComplexVisualizerExample : Visualizer<Msg>
    {
        public override object CreateDrawing(Msg msg, MessageMetadata meta)
        {
        }

        public override void DeleteDrawing(object drawing)
        {
			
        }

        public override System.Action CreateGUI(Msg msg, MessageMetadata meta, object drawing)
        {
			
        }
    }

- CreateDrawing will be called when your message appears in the HUD. You can do whatever you need to do in this function to display your visualization. The return value should be any value (for example a GameObject) that you can use to identify the graphic you just made.
- DeleteDrawing will be called when it's time to clean up the visualization, passing back the return value from CreateDrawing.
- CreateGUI is the same as described in the section above, except that it takes the object returned by CreateDrawing rather than a DebugDraw.Drawing.
- Note, this version of the class does not automatically select any colors or labels for you. If you want to use the automatic color selection logic, you can call `MessageVisualizations.PickColorForTopic(topic);`