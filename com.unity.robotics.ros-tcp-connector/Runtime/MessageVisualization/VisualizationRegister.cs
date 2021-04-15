using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public interface IVisualizer
    {
        object CreateDrawing(Message message, MessageMetadata meta, object oldDrawing);
        void DeleteDrawing(object drawing);
        Action CreateGUI(Message message, MessageMetadata meta, object drawing);
    }

    public struct MessageMetadata
    {
        public readonly string Topic;
        public readonly DateTime Timestamp;

        public MessageMetadata(string topic, DateTime timestamp)
        {
            Topic = topic;
            Timestamp = timestamp;
        }
    }

    public static class VisualizationRegister
    {
        private static Dictionary<string, Tuple<IVisualizer, int>> s_TopicVisualizers = new Dictionary<string, Tuple<IVisualizer, int>>();
        private static Dictionary<Type, Tuple<IVisualizer, int>> s_TypeVisualizers = new Dictionary<Type, Tuple<IVisualizer, int>>();

        public static void RegisterVisualizer<MsgType>(IVisualizer config, int priority = 0)
        {
            RegisterVisualizer(typeof(MsgType), config, priority);
        }

        public static void RegisterVisualizer(Type MsgType, IVisualizer config, int priority = 0)
        {
            Tuple<IVisualizer, int> currentEntry;
            if (!s_TypeVisualizers.TryGetValue(MsgType, out currentEntry) || currentEntry.Item2 <= priority)
            {
                s_TypeVisualizers[MsgType] = new Tuple<IVisualizer, int>(config, priority);
            }
        }

        public static void RegisterVisualizer(string topic, IVisualizer config, int priority = 0)
        {
            Tuple<IVisualizer, int> currentEntry;
            if (!s_TopicVisualizers.TryGetValue(topic, out currentEntry) || currentEntry.Item2 <= priority)
            {
                s_TopicVisualizers[topic] = new Tuple<IVisualizer, int>(config, priority);
            }
        }

        public static IVisualizer GetVisualizer(Message message, MessageMetadata meta)
        {
            Tuple<IVisualizer, int> result;
            s_TopicVisualizers.TryGetValue(meta.Topic, out result);
            if (result != null)
                return result.Item1;

            s_TypeVisualizers.TryGetValue(message.GetType(), out result);
            if (result != null)
                return result.Item1;

            return s_DefaultVisualizer;
        }

        class DefaultVisualizer : IVisualizer
        {
            // If you're trying to register the default visualizer, something has gone extremely wrong...
            public void Register(int priority) { throw new NotImplementedException(); }

            public object CreateDrawing(Message message, MessageMetadata meta, object oldDrawing) => null;

            public void DeleteDrawing(object drawing) { }

            public Action CreateGUI(Message message, MessageMetadata meta, object drawing)
            {
                string text = message.ToString();
                return () => { GUILayout.Label(text); };
            }
        }

        static DefaultVisualizer s_DefaultVisualizer = new DefaultVisualizer();
    }
}