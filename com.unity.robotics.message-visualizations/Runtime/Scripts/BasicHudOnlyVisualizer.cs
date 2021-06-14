using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BasicHudOnlyVisualizer<TargetMessageType> : MonoBehaviour, IVisualizer, IPriority
        where TargetMessageType : Message
    {
        [SerializeField]
        string m_Topic;

        public virtual void Start()
        {
            if (m_Topic == "")
                VisualizationRegistry.RegisterTypeVisualizer<TargetMessageType>(this, Priority);
            else
                VisualizationRegistry.RegisterTopicVisualizer(m_Topic, this, Priority);
        }

        public int Priority { get; set; }

        public bool CanShowDrawing => false;

        public virtual IMessageVisualization CreateVisualization(Message message, MessageMetadata meta, bool withGui, bool withDrawing)
        {
            var action = CreateGUI(message, meta, null);
            var vis = new BasicVisualization(message, meta, action, null);
            VisualizationRegistry.RegisterTopicVisualization(meta.Topic, vis);
            return vis;
        }

        public Action CreateGUI(Message message, MessageMetadata meta, object drawing)
        {
            if (!AssertMessageType(message, meta)) return MessageVisualizations.CreateDefaultGUI(message, meta);

            return CreateGUI((TargetMessageType)message, meta, (BasicDrawing)drawing);
        }

        public bool AssertMessageType(Message message, MessageMetadata meta)
        {
            if (!(message is TargetMessageType))
            {
                Debug.LogError($"{GetType()}, visualizer for topic \"{meta.Topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(TargetMessageType)}).");
                return false;
            }

            return true;
        }

        public virtual Action CreateGUI(TargetMessageType message, MessageMetadata meta, BasicDrawing drawing)
        {
            return MessageVisualizations.CreateDefaultGUI(message, meta);
        }
    }
}
