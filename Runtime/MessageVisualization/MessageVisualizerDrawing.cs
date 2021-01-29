using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using ROSGeometry;

public class MessageVisualizerDrawing<T> : ScriptableObject, IMessageVisualizer where T:RosMessageGeneration.Message
{
    DebugDraw.Drawing drawing;
    T message;

    public void Begin(RosMessageGeneration.Message msg)
    {
        message = (T)msg;
        drawing = DebugDraw.instance.CreateDrawing();
        Draw(drawing, message);
    }

    public virtual void Draw(DebugDraw.Drawing drawing, T message)
    {

    }

    public void Redraw()
    {
        drawing.Clear();
        Draw(drawing, message);
    }

    void IMessageVisualizer.DrawGUI(RosMessageGeneration.Message message)
    {
        DrawGUI(drawing, this.message);
    }

    public virtual void DrawGUI(DebugDraw.Drawing drawing, T message)
    {
        GUILayout.Label(message.ToString());
    }

    void IMessageVisualizer.End()
    {
        drawing.Destroy();
        Finish();
    }

    public virtual void Finish()
    {
    }
}
