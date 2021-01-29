using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Reflection;
using System;
using RosMessageGeneration;

public interface IMessageVisualizer
{
    void Begin(RosMessageGeneration.Message message);
    void DrawGUI(RosMessageGeneration.Message message);
    void End();
}

public class VisualizeMessageAttribute : System.Attribute
{
    public readonly System.Type messageType;
    public readonly string topic;

    public VisualizeMessageAttribute(System.Type messageType)
    {
        this.messageType = messageType;
    }

    public VisualizeMessageAttribute(string topic)
    {
        this.topic = topic;
    }

    static bool initialized;
    private static Dictionary<string, Type> TopicVisualizers = new Dictionary<string, Type>();
    private static Dictionary<Type, Type> TypeVisualizers = new Dictionary<Type, Type>();

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
                    if (attr.topic != null)
                        TopicVisualizers.Add(attr.topic, classType);
                    else
                        TypeVisualizers.Add(attr.messageType, classType);
                }
            }
        }
    }

    static System.Type GetVisualizerType(string topic, RosMessageGeneration.Message message)
    {
        System.Type result;
        if (TopicVisualizers.TryGetValue(topic, out result))
            return result;

        if (TypeVisualizers.TryGetValue(message.GetType(), out result))
            return result;

        return null;
    }

    public static IMessageVisualizer CreateVisualizer(string topic, RosMessageGeneration.Message message)
    {
        System.Type visualizerType = GetVisualizerType(topic, message);
        if(visualizerType != null)
        {
            IMessageVisualizer visualizer = (IMessageVisualizer)ScriptableObject.CreateInstance(visualizerType);
            visualizer.Begin(message);
            return visualizer;
        }
        
        if(defaultVisualizer == null)
        {
            defaultVisualizer = (DefaultVisualizer)ScriptableObject.CreateInstance(typeof(DefaultVisualizer));
        }
        return defaultVisualizer;
    }

    class DefaultVisualizer : ScriptableObject, IMessageVisualizer
    {
        public void Begin(Message message)
        {
        }

        public void DrawGUI(Message message)
        {
            GUILayout.Label(message.ToString());
        }

        public void End()
        {
        }
    }
    static DefaultVisualizer defaultVisualizer;
}
