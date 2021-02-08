# Message Visualization component

The ROSConnector HUD displays the last sent and received message, and also the request and response of the last completed service. The HUD should appear whenever your ROSConnector starts up. If it doesn't, ensure "Show HUD" is ticked in your ROS Settings window.

You can configure exactly how the HUD displays a message by using *custom visualizations*. You can use the preexisting visualizations, and also define your own.

# Static message visualizers

To create your own visualizer, start by making a class that implements the IMessageVisualizer interface.

	public interface IMessageVisualizer<MessageType> where MessageType:Message
	{
		void Begin(string topic, MessageType msg);
		void OnGUI();
		void End();
	}
	
- When the HUD needs to display a message, it will construct your class using the default constructor, then call the Begin function to tell it the name of the topic, and the message that's being displayed. Your Begin function should store this message - or whatever information it needs in order to display it - and create your visualization.
- OnGUI is a normal Unity OnGUI callback: it will be called once per UI event as long as your visualizer is active. You can use it to draw GUI elements that should appear in the HUD, and also to update your visualization if needed (for example, your gui could provide buttons to turn visualization elements on and off). Note this is a runtime GUI, not an editor GUI, so the GUI functions you call should come from GUILayout, not EditorGUILayout.
- Finally, the End function is called when the message has been deselected and your visualizer is about to be destroyed; you should use it to delete and clean up your visualization.

Here's a very simple example of a visualizer - this is the default visualizer class, used to display messages that have no other visualization defined:

    class DefaultVisualizer : IMessageVisualizer<Message>
    {
        string messageString;
        public void Begin(string topic, Message message)
        {
            this.messageString = message.ToString();
        }

        public void GUI()
        {
            GUILayout.Label(messageString);
        }

        public void End()
        {
        }
    }


# Registering Visualizers

There are two ways to register an IMessageVisualizer to display messages. First, you can put the RegisterVisualizer attribute on it.

	[RegisterVisualizer]
	class MyPointVisualizer: IMessageVisualizer<RosMessageTypes.Geometry.Point32>
	{
		//...
	}

This attribute marks this class as the default visualizer to use for displaying a message of the appropriate type (RosMessageTypes.Geometry.Point32 in this case). You don't need to to anything else, this attribute is enough to allow the HUD to find the class.
Optionally, the RegisterVisualizer attribute can also take a topic name argument:

	[RegisterVisualizer("special_points")]
	class MyPointVisualizer: IMessageVisualizer<RosMessageTypes.Geometry.Point32>
	{
		//...
	}

This marks the class as the visualizer to use when displaying messages sent or received on this topic. This takes precedence over the plain [RegisterVisualizer] attribute. Note that you can give a single class multiple RegisterVisualizer attributes, if you want to use it for several topics.

Finally, if you're trying to do something more complex, you can call this function from anywhere to dynamically declare a visualizer for a message type:

	MessageVisualizations.RegisterVisualizer<MyPointVisualizer, RosMessageTypes.Geometry.Point32>();

Like the attribute, the function can take an optional topic name:

	MessageVisualizations.RegisterVisualizer<MyPointVisualizer, RosMessageTypes.Geometry.Point32>(
		"special_points"
	);

# Supplying UserData to a visualizer

Often, it's useful to supply extra configuration parameters to a visualizer: especially if you want it to interact with specific objects in your scene. For this purpose, there's a variant of the IMessageVisualizer interface with an additional parameter, "userData".

	public interface IMessageVisualizer<MessageType, UserData> where MessageType:Message
	{
		void Begin(string topic, MessageType msg, UserData userData);
		void OnGUI();
		void End();
	}

The expected workflow is that you would create a class containing all the data your visualizer needs, to provide as its UserData parameter. Here's an example:

	public class UserDataVisualizerExample:
		IMessageVisualizer<RosMessageTypes.Geometry.Point32, UserDataVisualizerExample.MyUserData>
	{
		public class MyUserData
		{
			public string label;
			public Color color;
		}
		MyUserData userData;
		RosMessageTypes.Geometry.Point32 msg;
		
		void Begin(string topic, RosMessageTypes.Geometry.Point32 msg, MyUserData userData)
		{
			this.msg = msg;
			this.userData = userData;
		}
		
		void OnGUI()
		{
			Color oldColor = GUILayout.color;
			GUILayout.color = userData.color;
			GUILayout.Label(userData.label);
			GUILayout.Label(msg.ToString());
			GUILayout.color = oldColor;
		}
		
		void End()
		{
		}
	}

Then, to register a visualizer of this type, you'd supply an extra parameter to the RegisterVisualizer function:

	MessageVisualizations.RegisterVisualizer<UserDataVisualizerExample, RosMessageTypes.Geometry.Point32,
		UserDataVisualizerExample.MyUserData>
	(
		"special_points",
		new UserDataVisualizerExample.MyUserData
		{
			color = Color.red,
			label = "Hello"
		}
	);


# DebugDraw

For convenience, the MessageVisualizer folder also includes a simple DebugDraw class.

- To start using DebugDraw, call DebugDraw.CreateDrawing() to create a Drawing object. (Optionally, CreateDrawing can be given a maximum duration for the drawing to last, in seconds. If no duration is provided, it lasts until Destroyed).
- Once you have a Drawing object, you can draw graphics using its various functions such as DrawLine or DrawLabel. When you don't need the drawing any more, erase it by calling Destroy().
- The Drawing also has a Clear function. This is intended for users to change the drawing by calling Clear and then redrawing it. Don't simply Clear drawings when you mean to Destroy them; the system will continue to draw the Cleared drawing, at a small cost.
- Some of the functions on a Drawing allow you to input raw mesh data in the form of vertices and triangles; remember that in Unity, meshes have a clockwise winding order. (https://en.wikipedia.org/wiki/Back-face_culling)

Here's a simple example of a MessageVisualizer that uses the DebugDraw class to draw a dot and a label next to it:

	using System.Collections;
	using System.Collections.Generic;
	using UnityEngine;
	using Point = RosMessageTypes.Geometry.Point;
	using ROSGeometry;
	using RosMessageGeneration;

	[RegisterVisualizer] // use this class to visualize Point messages
	class MyPointVisualizer : IMessageVisualizer<Point>
	{
		Point message;
		DebugDraw.Drawing drawing;

		public void Begin(Message message)
		{
			this.message = (Point)message;
			drawing = DebugDraw.CreateDrawing();

			// convert from ROS coordinate space to Unity coordinate space using ROSGeometry.
			Vector3 position = this.message.From<FLU>();

			// Draw a point in red, with a 1 centimeter radius.
			drawing.DrawPoint(position, Color.red, 0.01f);
			
			// Draw the word "point" in red, 1.5 centimeters away from the point (in world space)
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