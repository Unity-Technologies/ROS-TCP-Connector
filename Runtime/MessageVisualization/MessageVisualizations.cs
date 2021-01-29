using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using ROSGeometry;

public static class MessageVisualizations<C> where C:CoordinateSpace, new()
{
    public static void DrawPoint(DebugDraw.Drawing drawing, Point message, Color color, string label, float size = 0.1f)
    {
        drawing.DrawPoint(message.From<C>(), color, size);
        drawing.DrawLabel(label, message.From<C>(), color, size*1.5f);
    }

    public static void DrawPoint32(DebugDraw.Drawing drawing, Point32 message, Color color, string label, float size = 0.1f)
    {
        drawing.DrawPoint(message.From<C>(), color, size);
        drawing.DrawLabel(label, message.From<C>(), color, size*1.5f);
    }

    public static void DrawVector3(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Vector3 message, Color color, string label, float size = 0.1f)
    {
        drawing.DrawPoint(message.From<C>(), color, size);
        drawing.DrawLabel(label, message.From<C>(), color, size*1.5f);
    }

    public static void DrawPose(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Pose message, Color color, string label, float size = 0.1f)
    {
        DrawPoint(drawing, message.position, color, label, size);
        UnityEngine.Vector3 point = message.position.From<C>();
        UnityEngine.Vector3 facing = message.orientation.From<C>() * UnityEngine.Vector3.forward;
        drawing.DrawLine(point, point + facing, color, size*0.5f);
    }

    public static void DrawQuaternion(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Quaternion message, UnityEngine.Vector3 position, float size = 0.1f)
    {
        UnityEngine.Quaternion quaternion = message.From<C>();
        UnityEngine.Vector3 right = quaternion * UnityEngine.Vector3.right * size;
        UnityEngine.Vector3 up = quaternion * UnityEngine.Vector3.up * size;
        UnityEngine.Vector3 forward = quaternion * UnityEngine.Vector3.forward * size;
        drawing.DrawLine(position, position + right, Color.red, size * 0.1f);
        drawing.DrawLine(position, position + up, Color.green, size * 0.1f);
        drawing.DrawLine(position, position + forward, Color.blue, size * 0.1f);
    }

    public static void DrawTransform(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Transform transform, float size = 0.1f)
    {
        DrawQuaternion(drawing, transform.rotation, transform.translation.From<C>(), size);
    }
}
