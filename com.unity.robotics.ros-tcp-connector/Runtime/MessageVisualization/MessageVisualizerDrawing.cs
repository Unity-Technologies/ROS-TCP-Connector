using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using ROSGeometry;

public class MessageVisualizerDrawing<T> : IMessageVisualizer where T:RosMessageGeneration.Message
{
    public DebugDraw.Drawing drawing;
    public string topic;
    public T message;

    public MessageVisualizerDrawing(string topic, RosMessageGeneration.Message msg)
    {
        drawing = DebugDraw.CreateDrawing();
        this.topic = topic;
        message = (T)msg;
        DrawVisual();
    }

    public virtual void DrawVisual()
    {

    }

    public virtual void GUI()
    {
        GUILayout.Label(message.ToString());
    }

    public virtual void End()
    {
        drawing.Destroy();
    }
}
