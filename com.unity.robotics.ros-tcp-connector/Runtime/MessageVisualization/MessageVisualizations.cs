using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System;
using System.Reflection;
using MessageVisualizerCreator = System.Func<string, Unity.Robotics.ROSTCPConnector.MessageGeneration.Message, IMessageVisualizerBase>;

public interface IMessageVisualizerBase
{
    void GUI();
    void End();
}

public interface IMessageVisualizer<Msg>: IMessageVisualizerBase where Msg:Message
{
    void Begin(string topic, Msg msg);
}

public interface IMessageVisualizer<Msg, UserData> : IMessageVisualizerBase where Msg:Message
{
    void Begin(string topic, Msg msg, UserData userData);
}

public static class MessageVisualizations
{
    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Point message, Color color, string label, float size = 0.1f) where C : ICoordinateSpace, new()
    {
        drawing.DrawPoint(message.From<C>(), color, size);
        drawing.DrawLabel(label, message.From<C>(), color, size*1.5f);
    }

    public static void GUI(string name, RosMessageTypes.Geometry.Point message)
    {
        GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
    }

    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Point32 message, Color color, string label, float size = 0.1f) where C : ICoordinateSpace, new()
    {
        drawing.DrawPoint(message.From<C>(), color, size);
        drawing.DrawLabel(label, message.From<C>(), color, size*1.5f);
    }

    public static void GUI(string name, RosMessageTypes.Geometry.Point32 message)
    {
        GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
    }

    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Vector3 message, Color color, string label, float size = 0.1f) where C : ICoordinateSpace, new()
    {
        drawing.DrawPoint(message.From<C>(), color, size);
        drawing.DrawLabel(label, message.From<C>(), color, size*1.5f);
    }

    public static void GUI(string name, RosMessageTypes.Geometry.Vector3 message)
    {
        GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
    }

    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Pose message, Color color, string label, float size = 0.1f) where C : ICoordinateSpace, new()
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

    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Quaternion message, UnityEngine.Vector3 position, float size = 0.1f) where C : ICoordinateSpace, new()
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

    public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Transform transform, float size = 0.1f) where C : ICoordinateSpace, new()
    {
        Draw<C>(drawing, transform.rotation, transform.translation.From<C>(), size);
    }

    public static void GUI(string name, RosMessageTypes.Geometry.Transform message)
    {
        GUI(name + " - Translation", message.translation);
        GUI("Rotation", message.rotation);
    }

    static bool initialized;
    private static Dictionary<string, MessageVisualizerCreator> TopicVisualizers = new Dictionary<string, MessageVisualizerCreator>();
    private static Dictionary<Type, MessageVisualizerCreator> TypeVisualizers = new Dictionary<Type, MessageVisualizerCreator>();
    private static Type[] emptyConstructorSignature = new Type[] { typeof(Message), typeof(string) };
    private static object[] emptyConstructorArgs = new object[] { };

    public static void InitAllVisualizers()
    {
        if (initialized)
            return;

        initialized = true;
        MethodInfo genericCreator = typeof(MessageVisualizations).GetMethod("GetCreatorTM");

        foreach (Assembly a in AppDomain.CurrentDomain.GetAssemblies())
        {
            foreach (Type classType in a.GetTypes())
            {
                if (!typeof(IMessageVisualizerBase).IsAssignableFrom(classType))
                    continue;

                foreach (VisualizeMessageAttribute attr in classType.GetCustomAttributes(typeof(VisualizeMessageAttribute), false))
                {
                    Type messageVisualizerGeneric = classType.GetInterface("IMessageVisualizer`1");
                    MethodInfo getCreator = genericCreator.MakeGenericMethod(new Type[] { classType, messageVisualizerGeneric.GenericTypeArguments[0] });
                    if (attr.topic != null)
                        TopicVisualizers.Add(attr.topic, (MessageVisualizerCreator)getCreator.Invoke(null, null));
                    else
                        TypeVisualizers.Add(attr.messageType, (MessageVisualizerCreator)getCreator.Invoke(null, null));
                }
            }
        }
    }

    public static void RegisterVisualizer<T,Msg>(string topic) where T:IMessageVisualizer<Msg>, new() where Msg:Message
    {
        TopicVisualizers.Add(topic, GetCreatorTM<T, Msg>());
    }

    public static void RegisterVisualizer<T,Msg>(System.Type messageType) where T : IMessageVisualizer<Msg>, new() where Msg : Message
    {
        TypeVisualizers.Add(messageType, GetCreatorTM<T, Msg>());
    }

    public static MessageVisualizerCreator GetCreatorTM<T, Msg>() where T : IMessageVisualizer<Msg>, new() where Msg:Message
    {
        return (string tpc, Message msg) =>
        {
            IMessageVisualizer<Msg> result = new T();
            result.Begin(tpc, (Msg)msg);
            return result;
        };
    }

    public static void RegisterVisualizer<T,Msg,U>(string topic, U userData) where T : IMessageVisualizer<Msg, U>, new() where Msg : Message
    {
        TopicVisualizers.Add(topic, GetCreatorTMU<T, Msg,U>(userData));
    }

    public static void RegisterVisualizer<T,Msg,U>(U userData) where T : IMessageVisualizer<Msg, U>, new() where Msg:Message
    {
        TypeVisualizers.Add(typeof(Msg), GetCreatorTMU<T, Msg,U>(userData));
    }

    private static MessageVisualizerCreator GetCreatorTMU<T, Msg,U>(U userData) where T : IMessageVisualizer<Msg, U>, new() where Msg:Message
    {
        return (string tpc, Message msg) =>
        {
            IMessageVisualizer<Msg, U> result = new T();
            result.Begin(tpc, (Msg)msg, userData);
            return result;
        };
    }

    public static IMessageVisualizerBase GetVisualizer(string topic, Message message)
    {
        MessageVisualizerCreator result;
        if (TopicVisualizers.TryGetValue(topic, out result))
            return result(topic, message);

        if (TypeVisualizers.TryGetValue(message.GetType(), out result))
            return result(topic, message);

        DefaultVisualizer defaultVisualizer = new DefaultVisualizer();
        defaultVisualizer.Begin(topic, message);
        return defaultVisualizer;
    }

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
}
