# Message Visualization component

The ROSConnector HUD displays the last sent and received message, and also the request and response of the last completed service. The HUD should appear whenever your ROSConnector starts up. If it doesn't, check whether "Show HUD" is ticked in your ROS Settings window.

For some messages, the HUD is able to display a custom visualization - and you can likewise define custom visualizations for your own messages.

# Writing a message visualizer

To create your own visualizer, start by making a class with the VisualizeMessage attribute on it. There are two variants of this attribute:

	[VisualizeMessage("some_topic")]

A "topic visualizer" for all messages sent or received on the given ROS topic.

	[VisualizeMessage(typeof(SomeMessageClass))]

A "message type visualizer" for all messages of the given type that don't have their own specific topic visualizer.

You can give a single class have multiple VisualizeMessage attributes - for example if you want the same visualizer to be used for several topics.

The class should also implement the IMessageVisualizer interface:

    public interface IMessageVisualizer
    {
        void Begin(RosMessageGeneration.Message message);
        void DrawGUI();
        void End();
    }

  - When the HUD needs to display a message of the given type/topic, it will instantiate your class, then call the Begin function to start displaying the given message. This function's argument is the message to display; it should store this message and do whatever setup you need to show your visualization.
  - DrawGUI is a standard Unity GUI callback, and will be called once per UI event as long as your visualizer is active. You can use it to draw GUI elements that should appear in the HUD, and also to update your visualization if needed.
  - Finally, the End function is called when the message has been deselected and your visualizer is about to be destroyed; you should use it to clean up your visualization.

Here's a very simple example - the default visualizer class, used to display messages that have no other visualization defined:

    class DefaultVisualizer : IMessageVisualizer
    {
        Message message;
		
        public void Begin(Message message)
        {
            this.message = message;
        }

        public void DrawGUI()
        {
            GUILayout.Label(message.ToString());
        }

        public void End()
        {
        }
    }

# DebugDraw

For convenience, the MessageVisualizer folder also includes a simple DebugDraw class. It's not specifically related to message visualization, but they do work well together.

To start using DebugDraw, call DebugDraw.CreateDrawing() to create a Drawing object. (Optionally, CreateDrawing can be given a maximum duration for the drawing to last, in seconds. If no duration is provided, it lasts until Destroyed).
  Once you have a Drawing object, you can draw graphics using its various functions such as DrawLine or DrawLabel. When you don't need the drawing any more, erase it by calling Destroy().
  The Drawing also has a Clear function. This is intended for users to change the drawing by calling Clear and then redrawing it. Don't simply Clear drawings when you mean to Destroy them; the system will continue to draw the Cleared drawing, at a small cost.
  Some of the functions on a Drawing allow you to input raw mesh data in the form of vertices and triangles; remember that in Unity, meshes have a clockwise winding order.

Here's a simple example of a MessageVisualizer that uses the DebugDraw class to draw a dot and a label next to it:

	using System.Collections;
	using System.Collections.Generic;
	using UnityEngine;
	using Point = RosMessageTypes.Geometry.Point;
	using ROSGeometry;
	using RosMessageGeneration;

	[VisualizeMessage(typeof(Point))] // use this class to visualize Point messages
	class MyPointVisualizer : IMessageVisualizer
	{
		Point message;
		DebugDraw.Drawing drawing;

		public void Begin(Message message)
		{
			this.message = (Point)message;
			Vector3 position = this.message.From<FLU>(); // see ROSGeometry for coordinate conversion details
			drawing = DebugDraw.CreateDrawing();

			drawing.DrawPoint(position, Color.red, 0.01f);
			drawing.DrawLabel("point", position, Color.red, 0.015f);
		}

		public void DrawGUI()
		{
			GUILayout.Label(message.ToString());
		}

		public void End()
		{
			drawing.Destroy();
		}
	}