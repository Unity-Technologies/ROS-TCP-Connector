using System;
using System.Collections.Generic;
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
        bool hasDrawing { get; }
        bool hasAction { get; }
        void DeleteDrawing();
        void OnGUI();
        void CreateDrawing();
        void Recycle(IVisual oldVisual);
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
        static Dictionary<string, Tuple<IVisualFactory, int>> s_TopicVisualFactories = new Dictionary<string, Tuple<IVisualFactory, int>>();
        static Dictionary<string, Tuple<IVisualFactory, int>> s_TypeVisualFactories = new Dictionary<string, Tuple<IVisualFactory, int>>();

        static DefaultVisualFactory s_DefaultVisualFactory = new DefaultVisualFactory();

        public static void RegisterTypeVisualizer<MsgType>(IVisualFactory visualFactory, int priority = 0) where MsgType : Message
        {
            RegisterTypeVisualizer(MessageRegistry.GetMessageName<MsgType>(), visualFactory, priority);
        }

        public static void RegisterTypeVisualizer(string rosMessageName, IVisualFactory visualFactory, int priority = 0)
        {
            Tuple<IVisualFactory, int> currentEntry;
            if (!s_TypeVisualFactories.TryGetValue(rosMessageName, out currentEntry) || currentEntry.Item2 <= priority) s_TypeVisualFactories[rosMessageName] = new Tuple<IVisualFactory, int>(visualFactory, priority);
        }

        public static void RegisterTopicVisualizer(string topic, IVisualFactory visualFactory, int priority = 0)
        {
            Tuple<IVisualFactory, int> currentEntry;
            if (!s_TopicVisualFactories.TryGetValue(topic, out currentEntry) || currentEntry.Item2 <= priority) s_TopicVisualFactories[topic] = new Tuple<IVisualFactory, int>(visualFactory, priority);
        }

        public static IVisualFactory GetVisualizer(string topic, string rosMessageName=null)
        {
            Tuple<IVisualFactory, int> result;
            s_TopicVisualFactories.TryGetValue(topic, out result);
            if (result != null)
                return result.Item1;

            if (rosMessageName != null)
            {
                s_TypeVisualFactories.TryGetValue(rosMessageName, out result);
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
            s_TopicVisualFactories.TryGetValue(meta.Topic, out result);
            if (result != null)
                return result.Item1;

            s_TypeVisualFactories.TryGetValue(message.RosMessageName, out result);
            if (result != null)
                return result.Item1;

            return s_DefaultVisualFactory;
        }

        class DefaultVisualFactory : IVisualFactory
        {
            public IVisual CreateVisual(Message message, MessageMetadata meta)
            {
                return new DefaultVisual(message, meta);
            }

            // If you're trying to register the default visualFactory, something has gone extremely wrong...
            public void Register(int priority) { throw new NotImplementedException(); }
            public bool CanShowDrawing => false;
        }

        class DefaultVisual : IVisual
        {
            public Message message { get; }
            public MessageMetadata meta { get; }
            public bool hasDrawing => false;
            public bool hasAction => true;

            public DefaultVisual(Message newMessage, MessageMetadata newMeta)
            {
                message = newMessage;
                meta = newMeta;
            }

            public void OnGUI()
            {
                string text = message.ToString();
                GUILayout.Label(text);
            }

            public void CreateDrawing() { }
            public void DeleteDrawing() { }
            public void Recycle(IVisual oldVisual) { }
        }
    }
}
