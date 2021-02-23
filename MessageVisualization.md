# Message Visualization component

In the ROSConnector HUD you can see the last sent (publisher) and received (subscriber) message, and also the request and response of the last completed service. The HUD should appear whenever your ROSConnector starts up. If it doesn't, ensure "Show HUD" is ticked in your ROS Settings window.

You can configure exactly how the HUD displays a message by setting up your own Visualization Suite, and/or creating your own *custom visualizations*.

# Configuring your visualization suite

The simplest way to tune how visualizations appear is to set up your own Visualization Suite. This is a collection of message visualizers, placed in your scene, that control how messages are displayed in that scene. (If you need to share your visualizers between multiple scenes, consider creating a prefab.)

To create a visualization suite, create a new GameObject in your scene, call it "VisualizationSuite" (or whatever you want) and add any number of visualizer components to it. Each visualizer can be set up to apply only to messages on a specific topic - just edit the "topic" field - and can have its own custom color and label.

For example, let's assume you want to call out Point messages on the "important_points" topic, and you'd like them to appear extra large and in red. So maybe you'd add a DefaultVisualizerPoint to your suite, and set it to draw large red points.

![](images~/VisualizationSuiteExample.png)

(Note that in Unity, "size" is in Unity coordinates, so a size of 1 means the points have a radius of 1 meter.)

# Writing a message visualizer using DebugDraw

Although the message visualization system includes default visualizers for many common message types, inevitably you'll probably have your own messages you want to visualize, or you'll want to change how a message is displayed for your project's specific needs. The simplest way to create a new visualizer is to write a MonoBehaviour script that inherits from BasicVisualizer. Here's a simple example:

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
- Note that, in principle, your visualizer class should be able to visualize more than one message at the same time. You should not store information about "the current message" in the Visualizer class itself, because there isn't just one current message. That kind of information can be stored as a local variable in your CreateGUI function.

# Writing a custom message visualizer

If you need to create a visualization more complex than the simple DebugDraw class can support, you can write your own MonoBehaviour script that implements the IVisualizer interface.

	public interface IVisualizer
	{
		object CreateDrawing(Message msg, MessageMetadata meta);
		void DeleteDrawing(object drawing);
		Action CreateGUI(Message msg, MessageMetadata meta, object drawing);
	}

Here's a simple example of a visualizer that instantiates a prefab to mark the position and rotation of a Transform message:

	using UnityEngine;
	using Unity.Robotics.ROSTCPConnector.ROSGeometry;
	using Unity.Robotics.ROSTCPConnector.MessageGeneration;
	using MTransform = RosMessageTypes.Geometry.Transform;
	using Unity.Robotics.MessageVisualizers;

	public class PrefabTransformVisualizer : MonoBehaviour, IVisualizer
	{
		public string topic;
		public GameObject markerPrefab;

		public void Start()
		{
			// For a Visualizer to actually be known to the HUD,
			// it must call MessageVisualizations.RegisterVisualizer.
			// There are two versions: either register for messages on a specific topic,
			// or register for all messages of a specific type.
			// In this example, we choose between those two options based on whether
			// a topic has been specified.
			if (topic != "")
				MessageVisualizations.RegisterVisualizer(topic, this);
			else
				MessageVisualizations.RegisterVisualizer<MTransform>(this);
		}

		public object CreateDrawing(Message message, MessageMetadata meta)
		{
			// instantiate a prefab and put it at the appropriate position and rotation.
			MTransform transformMessage = (MTransform)message;
			GameObject marker = Instantiate(markerPrefab);
			marker.transform.position = transformMessage.translation.From<FLU>();
			marker.transform.rotation = transformMessage.rotation.From<FLU>();
			return marker;
		}

		public void DeleteDrawing(object drawing)
		{
			// destroy the instance when we're done
			GameObject.Destroy((GameObject)drawing);
		}

		public System.Action CreateGUI(Message message, MessageMetadata meta, object drawing)
		{
			return () =>
			{
				// call the standard GUI function for displaying Transform messages
				MessageVisualizations.GUI(meta.topic, (MTransform)message);
			};
		}
	}

- If your visualizer doesn't call MessageVisualizations.RegisterVisualizer, it won't get used.
- CreateDrawing will be called to display graphics for your message. You can do whatever you need to do in this function to display your visualization. The return value should be any value (a GameObject in this case, but it really can be anything - an index in a list, for example) that you can later use to identify the graphic you just made.
- DeleteDrawing will be called when it's time to clean up the visualization, passing the value you returned from CreateDrawing.
- CreateGUI is as described in the section above, except that its first argument is the base Message class, and the last argument is whatever object was returned by CreateDrawing.
- Note, unlike the BasicVisualizer class, creating a visualizer this way does not automatically select a color for your visualization. To use the same automatic color selection logic as before, you can call `MessageVisualizations.PickColorForTopic(topic);`
