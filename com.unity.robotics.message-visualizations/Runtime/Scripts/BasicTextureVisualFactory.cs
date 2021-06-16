using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BasicTextureVisualFactory<TargetMessageType> : MonoBehaviour, IVisualFactory, IPriority
        where TargetMessageType : Message
    {
        [SerializeField]
        string m_Topic;

        public virtual void Start()
        {
            if (m_Topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<TargetMessageType>(this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(m_Topic, this, Priority);
            }
        }

        public int Priority { get; set; }
        public bool CanShowDrawing => false;

        public IVisual CreateVisual(Message message, MessageMetadata meta)
        {
            return new BasicTextureVisualization<TargetMessageType>((TargetMessageType)message, meta, this, CreateTexture((TargetMessageType)message));
        }

        public abstract Texture2D CreateTexture(TargetMessageType message);

        public virtual Action CreateGUI(TargetMessageType message, MessageMetadata meta, Texture2D tex)
        {
            return MessageVisualizations.CreateDefaultGUI(message, meta);
        }
    }
}
