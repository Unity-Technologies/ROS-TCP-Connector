using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class MultiDrawingVisualFactory<T> : MonoBehaviour, IVisualFactory, IPriority
        where T : Message
    {
        [SerializeField]
        string m_Topic;
        public string Topic { get => m_Topic; set => m_Topic = value; }

        [SerializeField]
        int m_HistoryLength;
        public int HistoryLength { get => m_HistoryLength; set => m_HistoryLength = value; }

        public virtual void Start()
        {
            if (string.IsNullOrEmpty(m_Topic))
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<T>(this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(m_Topic, this, Priority);
            }
        }

        public int Priority { get; set; }

        public bool CanShowDrawing => true;

        public virtual IVisual CreateVisual()
        {
            return new MultiDrawingVisual<T>(this, m_HistoryLength);
        }

        public virtual void Draw(BasicDrawing drawing, IEnumerable<Tuple<T, MessageMetadata>> messages) { }

        public virtual Action CreateGUI(IEnumerable<Tuple<T, MessageMetadata>> messages)
        {
            List<Action> actions = new List<Action>();
            foreach ((T message, MessageMetadata meta) in messages)
            {
                actions.Add(CreateGUI(message, meta));
            }
            return () => actions.ForEach(a => a());
        }

        public virtual Action CreateGUI(T message, MessageMetadata meta)
        {
            return MessageVisualizationUtils.CreateDefaultGUI(message, meta);
        }
    }
}
