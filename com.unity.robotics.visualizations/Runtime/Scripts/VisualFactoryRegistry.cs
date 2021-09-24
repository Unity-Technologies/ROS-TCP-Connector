using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public readonly struct MessageMetadata
    {
        public readonly string Topic;
        public readonly float FrameTime;
        public readonly DateTime Timestamp;

        public MessageMetadata(string topic, float frameTime, DateTime timestamp)
        {
            Topic = topic;
            FrameTime = frameTime;
            Timestamp = timestamp;
        }
    }

    public static class VisualFactoryRegistry
    {
        class PrioritizedList<T>
        {
            public T Best { get; private set; }
            int m_BestPriority = int.MinValue;
            List<T> m_All = new List<T>();
            public IEnumerable<T> All => m_All;

            public void Add(T value, int priority)
            {
                m_All.Add(value);
                if (m_BestPriority <= priority)
                {
                    m_BestPriority = priority;
                    Best = value;
                }
            }
        }

        static Dictionary<string, PrioritizedList<IVisualFactory>> s_TopicVisualFactories = new Dictionary<string, PrioritizedList<IVisualFactory>>();
        static Dictionary<string, PrioritizedList<IVisualFactory>> s_TypeVisualFactories = new Dictionary<string, PrioritizedList<IVisualFactory>>();

        static ToStringVisualizer s_DefaultVisualFactory = new ToStringVisualizer();

        public static void RegisterTypeVisualizer<MsgType>(IVisualFactory visualFactory, int priority = 0) where MsgType : Message
        {
            RegisterTypeVisualizer(MessageRegistry.GetRosMessageName<MsgType>(), visualFactory, priority);
        }

        public static void RegisterTypeVisualizer(string rosMessageName, IVisualFactory visualFactory, int priority = 0)
        {
            PrioritizedList<IVisualFactory> currentEntry;
            if (!s_TypeVisualFactories.TryGetValue(rosMessageName, out currentEntry))
            {
                currentEntry = new PrioritizedList<IVisualFactory>();
                currentEntry.Add(s_DefaultVisualFactory, int.MinValue);
                s_TypeVisualFactories[rosMessageName] = currentEntry;
            }
            currentEntry.Add(visualFactory, priority);
        }

        public static void RegisterTopicVisualizer(string topic, IVisualFactory visualFactory, int priority = 0)
        {
            if (topic == null)
                Debug.Log("Registered null topic!");
            PrioritizedList<IVisualFactory> currentEntry;
            if (!s_TopicVisualFactories.TryGetValue(topic, out currentEntry))
            {
                currentEntry = new PrioritizedList<IVisualFactory>();
                s_TopicVisualFactories[topic] = currentEntry;
            }
            currentEntry.Add(visualFactory, priority);
        }

        public static IVisualFactory GetVisualFactory(string topic, string rosMessageName = null)
        {
            PrioritizedList<IVisualFactory> result;
            s_TopicVisualFactories.TryGetValue(topic, out result);
            if (result != null)
                return result.Best;

            if (rosMessageName != null)
            {
                s_TypeVisualFactories.TryGetValue(rosMessageName, out result);
                if (result != null)
                    return result.Best;

                if (MessageRegistry.GetDeserializeFunction(rosMessageName) != null)
                    return s_DefaultVisualFactory;
            }

            return null;
        }

        public static IEnumerable<IVisualFactory> GetAllVisualFactories(string topic, string rosMessageName)
        {
            PrioritizedList<IVisualFactory> result;
            IEnumerable<IVisualFactory> topicVisualizers = null;
            if (topic != null)
            {
                s_TopicVisualFactories.TryGetValue(topic, out result);
                if (result != null)
                    topicVisualizers = result.All;
            }

            IEnumerable<IVisualFactory> typeVisualizers = null;
            if (rosMessageName != null)
            {
                s_TypeVisualFactories.TryGetValue(rosMessageName, out result);
                if (result != null)
                    typeVisualizers = result.All;
                else if (MessageRegistry.GetDeserializeFunction(rosMessageName) != null)
                    typeVisualizers = new IVisualFactory[] { s_DefaultVisualFactory };
            }

            if (topicVisualizers == null)
            {
                return typeVisualizers;
            }

            if (typeVisualizers != null)
                return topicVisualizers.Concat(typeVisualizers);
            else
                return topicVisualizers;
        }
    }
}
