using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public interface IVisualizer
    {
        bool CanShowDrawing { get; }
        object CreateDrawing(Message message, MessageMetadata meta, object oldDrawing);
        void DeleteDrawing(object drawing);
        Action CreateGUI(Message message, MessageMetadata meta, object drawing);
    }

    public struct MessageMetadata
    {
        public readonly string Topic;
        public readonly int FrameIndex;
        public readonly DateTime Timestamp;

        public MessageMetadata(string topic, int frameIndex, DateTime timestamp)
        {
            Topic = topic;
            FrameIndex = frameIndex;
            Timestamp = timestamp;
        }
    }

    public static class VisualizationRegistry
    {
        private static Dictionary<string, Tuple<IVisualizer, int>> s_TopicVisualizers = new Dictionary<string, Tuple<IVisualizer, int>>();
        private static Dictionary<string, Tuple<IVisualizer, int>> s_TypeVisualizers = new Dictionary<string, Tuple<IVisualizer, int>>();

        public static void RegisterTypeVisualizer<MsgType>(IVisualizer visualizer, int priority = 0) where MsgType:Message
        {
            RegisterTypeVisualizer(MessageRegistry.GetMessageName<MsgType>(), visualizer, priority);
        }

        public static void RegisterTypeVisualizer(string rosMessageName, IVisualizer visualizer, int priority = 0)
        {
            Tuple<IVisualizer, int> currentEntry;
            if (!s_TypeVisualizers.TryGetValue(rosMessageName, out currentEntry) || currentEntry.Item2 <= priority)
            {
                s_TypeVisualizers[rosMessageName] = new Tuple<IVisualizer, int>(visualizer, priority);
            }
        }

        public static void RegisterTopicVisualizer(string topic, IVisualizer visualizer, int priority = 0)
        {
            Tuple<IVisualizer, int> currentEntry;
            if (!s_TopicVisualizers.TryGetValue(topic, out currentEntry) || currentEntry.Item2 <= priority)
            {
                s_TopicVisualizers[topic] = new Tuple<IVisualizer, int>(visualizer, priority);
            }
        }

        public static IVisualizer GetVisualizer(string topic, string rosMessageName)
        {
            Tuple<IVisualizer, int> result;
            s_TopicVisualizers.TryGetValue(topic, out result);
            if (result != null)
                return result.Item1;

            if (rosMessageName != null)
            {
                s_TypeVisualizers.TryGetValue(rosMessageName, out result);
                if (result != null)
                    return result.Item1;

                if (MessageRegistry.GetConstructor(rosMessageName) != null)
                    return s_DefaultVisualizer;
            }

            return null;
        }

        public static IVisualizer GetVisualizer(Message message, MessageMetadata meta)
        {
            Tuple<IVisualizer, int> result;
            s_TopicVisualizers.TryGetValue(meta.Topic, out result);
            if (result != null)
                return result.Item1;

            s_TypeVisualizers.TryGetValue(message.RosMessageName, out result);
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

            public bool CanShowDrawing => false;
        }

        static DefaultVisualizer s_DefaultVisualizer = new DefaultVisualizer();
    }
}