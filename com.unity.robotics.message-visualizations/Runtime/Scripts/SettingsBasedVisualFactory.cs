using System;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class SettingsBasedVisualFactory<TMessageType, TDrawingSettings> : MonoBehaviour, IVisualFactory, IPriority
        where TMessageType : Message
        where TDrawingSettings : VisualizerSettings<TMessageType>
    {
        [SerializeField]
        string m_Topic;
        public string Topic { get => m_Topic; set => m_Topic = value; }

        [SerializeField]
        TDrawingSettings m_Settings;
        public TDrawingSettings Settings { get => m_Settings; set => m_Settings = value; }

        public virtual void Start()
        {
            if (m_Topic == null || m_Topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer(MessageRegistry.GetRosMessageName<TMessageType>(), this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(m_Topic, this, Priority);
            }
        }

        public bool AssertMessageType(Message message, MessageMetadata meta)
        {
            if (!(message is TMessageType))
            {
                Debug.LogError($"{GetType()}, DrawingSettings for topic \"{meta.Topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(TMessageType)}).");
                return false;
            }

            return true;
        }

        public IVisual CreateVisual(Message message, MessageMetadata meta)
        {
            if (!AssertMessageType(message, meta))
                return null;

            return new DrawingVisual<TMessageType>((TMessageType)message, meta, m_Settings);
        }

        public void Draw(BasicDrawing drawing, TMessageType message, MessageMetadata meta)
        {
            m_Settings.Draw(drawing, message, meta);
        }

        public void Redraw()
        {
            // settings have changed - update the visualization
            IVisualFactory thisFactory = this;
            foreach (TopicVisualizationState topic in HUDPanel.AllTopics.Values)
            {
                if (topic == null)
                    continue;
                var contents = topic.WindowContents;
                if (contents != null && contents.HasDrawing && contents.VisualFactory == thisFactory)
                {
                    contents.GetVisual().CreateDrawing();
                }
            }
        }

        public int Priority { get; set; }

        public bool CanShowDrawing => true;
    }
}
