using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using ROSGeometry;
using System;
using System.Reflection;
using CreateMessageVisualizer = System.Func<string, RosMessageGeneration.Message, IMessageVisualizer>;

public interface IMessageVisualizer
{
    void GUI();
    void End();
}

public static class MessageVisualizations
{
    public static void Draw<C>(DebugDraw.Drawing drawing, Point message, Color color, string label, float size = 0.1f) where C : CoordinateSpace, new()
    {
        drawing.DrawPoint(message.From<C>(), color, size);
        drawing.DrawLabel(label, message.From<C>(), color, size*1.5f);
    }

    public static void GUI(string name, Point message)
    {
        GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
    }

    public static void Draw<C>(DebugDraw.Drawing drawing, Point32 message, Color color, string label, float size = 0.1f) where C : CoordinateSpace, new()
    {
        drawing.DrawPoint(message.From<C>(), color, size);
        drawing.DrawLabel(label, message.From<C>(), color, size*1.5f);
    }

    public static void GUI(string name, Point32 message)
    {
        GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
    }

    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Vector3 message, Color color, string label, float size = 0.1f) where C : CoordinateSpace, new()
    {
        drawing.DrawPoint(message.From<C>(), color, size);
        drawing.DrawLabel(label, message.From<C>(), color, size*1.5f);
    }

    public static void GUI(string name, RosMessageTypes.Geometry.Vector3 message)
    {
        GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
    }

    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Pose message, Color color, string label, float size = 0.1f) where C : CoordinateSpace, new()
    {
        Draw<C>(drawing, message.position, color, label, size);
        UnityEngine.Vector3 point = message.position.From<C>();
        UnityEngine.Vector3 facing = message.orientation.From<C>() * UnityEngine.Vector3.forward;
        drawing.DrawLine(point, point + facing, color, size*0.5f);
    }

    public static void GUI(string name, RosMessageTypes.Geometry.Pose message)
    {
        GUI(name+" - Position", message.position);
        GUI("Orientation", message.orientation);
    }

    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Quaternion message, UnityEngine.Vector3 position, float size = 0.1f) where C : CoordinateSpace, new()
    {
        UnityEngine.Quaternion quaternion = message.From<C>();
        UnityEngine.Vector3 right = quaternion * UnityEngine.Vector3.right * size;
        UnityEngine.Vector3 up = quaternion * UnityEngine.Vector3.up * size;
        UnityEngine.Vector3 forward = quaternion * UnityEngine.Vector3.forward * size;
        drawing.DrawLine(position, position + right, Color.red, size * 0.1f);
        drawing.DrawLine(position, position + up, Color.green, size * 0.1f);
        drawing.DrawLine(position, position + forward, Color.blue, size * 0.1f);
    }

    public static void GUI(string name, RosMessageTypes.Geometry.Quaternion message)
    {
        GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}, {message.w:F2}]");
    }

    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Transform transform, float size = 0.1f) where C : CoordinateSpace, new()
    {
        Draw<C>(drawing, transform.rotation, transform.translation.From<C>(), size);
    }

    public static void GUI(string name, RosMessageTypes.Geometry.Transform message)
    {
        GUI(name + " - Translation", message.translation);
        GUI("Rotation", message.rotation);
    }

    static bool initialized;
    private static Dictionary<string, CreateMessageVisualizer> TopicVisualizers = new Dictionary<string, CreateMessageVisualizer>();
    private static Dictionary<Type, CreateMessageVisualizer> TypeVisualizers = new Dictionary<Type, CreateMessageVisualizer>();
    private static Type[] visualizerConstructorSignature = new Type[] { typeof(RosMessageGeneration.Message), typeof(string) };

    public static void InitAllVisualizers()
    {
        if (initialized)
            return;

        initialized = true;

        foreach (Assembly a in AppDomain.CurrentDomain.GetAssemblies())
        {
            foreach (Type classType in a.GetTypes())
            {
                if (!typeof(IMessageVisualizer).IsAssignableFrom(classType))
                    continue;

                foreach (VisualizeMessageAttribute attr in classType.GetCustomAttributes(typeof(VisualizeMessageAttribute), false))
                {
                    ConstructorInfo constructor = classType.GetConstructor(visualizerConstructorSignature);
                    CreateMessageVisualizer visualize = (string topic, RosMessageGeneration.Message msg) => (IMessageVisualizer)constructor.Invoke(new object[] { topic, msg });
                    if (attr.topic != null)
                        TopicVisualizers.Add(attr.topic, visualize);
                    else
                        TypeVisualizers.Add(attr.messageType, visualize);
                }
            }
        }
    }

    public static void RegisterVisualizer(this CreateMessageVisualizer visualizer, string topic)
    {
        TopicVisualizers.Add(topic, visualizer);
    }

    public static void RegisterVisualizer(this CreateMessageVisualizer visualizer, System.Type messageType)
    {
        TypeVisualizers.Add(messageType, visualizer);
    }

    public static IMessageVisualizer GetVisualizer(string topic, RosMessageGeneration.Message message)
    {
        CreateMessageVisualizer result;
        if (TopicVisualizers.TryGetValue(topic, out result))
            return result(topic, message);

        if (TypeVisualizers.TryGetValue(message.GetType(), out result))
            return result(topic, message);

        return new DefaultVisualizer(message.ToString());
    }

    class DefaultVisualizer : IMessageVisualizer
    {
        string messageString;
        public DefaultVisualizer(string messageString)
        {
            this.messageString = messageString;
        }

        public void GUI()
        {
            GUILayout.Label(messageString);
        }

        public void End()
        {
        }
    }
}
