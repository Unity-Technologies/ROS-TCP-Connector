using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public interface IVisualizer
    {
        object CreateDrawing(Message message, MessageMetadata meta);
        void DeleteDrawing(object drawing);
        Action CreateGUI(Message message, MessageMetadata meta, object drawing);
    }

    public struct MessageMetadata
    {
        public readonly string topic;
        public readonly DateTime timestamp;

        public MessageMetadata(string topic, DateTime timestamp)
        {
            this.topic = topic;
            this.timestamp = timestamp;
        }
    }

    public static class VisualizationRegister
    {
        private static Dictionary<string, Tuple<IVisualizer, int>> TopicVisualizers = new Dictionary<string, Tuple<IVisualizer, int>>();
        private static Dictionary<Type, Tuple<IVisualizer, int>> TypeVisualizers = new Dictionary<Type, Tuple<IVisualizer, int>>();

        public static void RegisterVisualizer<MsgType>(IVisualizer config, int priority = 0)
        {
            RegisterVisualizer(typeof(MsgType), config, priority);
        }

        public static void RegisterVisualizer(Type MsgType, IVisualizer config, int priority = 0)
        {
            Tuple<IVisualizer, int> currentEntry;
            if (!TypeVisualizers.TryGetValue(MsgType, out currentEntry) || currentEntry.Item2 <= priority)
            {
                TypeVisualizers[MsgType] = new Tuple<IVisualizer, int>(config, priority);
            }
        }

        public static void RegisterVisualizer(string topic, IVisualizer config, int priority = 0)
        {
            Tuple<IVisualizer, int> currentEntry;
            if (!TopicVisualizers.TryGetValue(topic, out currentEntry) || currentEntry.Item2 <= priority)
            {
                TopicVisualizers[topic] = new Tuple<IVisualizer, int>(config, priority);
            }
        }

        public static IVisualizer GetVisualizer(Message message, MessageMetadata meta)
        {
            Tuple<IVisualizer, int> result;
            TopicVisualizers.TryGetValue(meta.topic, out result);
            if (result != null)
                return result.Item1;

            TypeVisualizers.TryGetValue(message.GetType(), out result);
            if (result != null)
                return result.Item1;

            return defaultVisualizer;
        }

        class DefaultVisualizer : IVisualizer
        {
            // If you're trying to register the default visualizer, something has gone extremely wrong...
            public void Register(int priority) { throw new NotImplementedException(); }

            public object CreateDrawing(Message message, MessageMetadata meta) => null;

            public void DeleteDrawing(object drawing) { }

            public Action CreateGUI(Message message, MessageMetadata meta, object drawing)
            {
                string text = message.ToString();
                return () => { GUILayout.Label(text); };
            }
        }

        static DefaultVisualizer defaultVisualizer = new DefaultVisualizer();
    }
}