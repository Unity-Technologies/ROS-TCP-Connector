using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public interface IVisualFactory
    {
        bool CanShowDrawing { get; }
        IVisual CreateVisual(Message message, MessageMetadata meta);
    }

    public interface IVisual
    {
        Message message { get; }
        MessageMetadata meta { get; }
        bool hasDrawing { get; set; }
        bool hasAction { get; set; }
        void DeleteDrawing();
        void OnGUI();
        void CreateDrawing();
    }

    public interface ITextureVisual : IVisual
    {
        Texture2D GetTexture();
    }

    public readonly struct MessageMetadata
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

    public static class VisualFactoryRegistry
    {
        static Dictionary<string, Tuple<IVisualFactory, int>> s_TopicVisualizers = new Dictionary<string, Tuple<IVisualFactory, int>>();
        static Dictionary<string, Tuple<IVisualFactory, int>> s_TypeVisualizers = new Dictionary<string, Tuple<IVisualFactory, int>>();

        static DefaultVisualizer s_DefaultVisualFactory = new DefaultVisualizer();

        public static void RegisterTypeVisualizer<MsgType>(IVisualFactory visualFactory, int priority = 0) where MsgType : Message
        {
            RegisterTypeVisualizer(MessageRegistry.GetMessageName<MsgType>(), visualFactory, priority);
        }

        public static void RegisterTypeVisualizer(string rosMessageName, IVisualFactory visualFactory, int priority = 0)
        {
            Tuple<IVisualFactory, int> currentEntry;
            if (!s_TypeVisualizers.TryGetValue(rosMessageName, out currentEntry) || currentEntry.Item2 <= priority) s_TypeVisualizers[rosMessageName] = new Tuple<IVisualFactory, int>(visualFactory, priority);
        }

        public static void RegisterTopicVisualizer(string topic, IVisualFactory visualFactory, int priority = 0)
        {
            Tuple<IVisualFactory, int> currentEntry;
            if (!s_TopicVisualizers.TryGetValue(topic, out currentEntry) || currentEntry.Item2 <= priority) s_TopicVisualizers[topic] = new Tuple<IVisualFactory, int>(visualFactory, priority);
        }

        public static IVisualFactory GetVisualizer(string topic, string rosMessageName=null)
        {
            Tuple<IVisualFactory, int> result;
            s_TopicVisualizers.TryGetValue(topic, out result);
            if (result != null)
                return result.Item1;

            if (rosMessageName != null)
            {
                s_TypeVisualizers.TryGetValue(rosMessageName, out result);
                if (result != null)
                    return result.Item1;

                if (MessageRegistry.GetConstructor(rosMessageName) != null)
                    return s_DefaultVisualFactory;
            }

            return null;
        }

        public static IVisualFactory GetVisualizer(Message message, MessageMetadata meta)
        {
            Tuple<IVisualFactory, int> result;
            s_TopicVisualizers.TryGetValue(meta.Topic, out result);
            if (result != null)
                return result.Item1;

            s_TypeVisualizers.TryGetValue(message.RosMessageName, out result);
            if (result != null)
                return result.Item1;

            return s_DefaultVisualFactory;
        }
        
        class DefaultVisualizer : IVisualFactory
        {
            public IVisual CreateVisual(Message message, MessageMetadata meta)
            {
                return null;
            }

            public bool CanShowDrawing => false;

            // If you're trying to register the default visualFactory, something has gone extremely wrong...
            public void Register(int priority)
            {
                throw new NotImplementedException();
            }

            public object CreateDrawing(Message message, MessageMetadata meta, object oldDrawing)
            {
                return null;
            }

            public void DeleteDrawing(object drawing) { }

            public Action CreateGUI(Message message, MessageMetadata meta, object drawing)
            {
                var text = message.ToString();
                return () => { GUILayout.Label(text); };
            }
        }
    }
}
