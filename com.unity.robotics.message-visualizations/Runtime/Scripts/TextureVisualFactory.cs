using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class TextureVisualFactory<T> : MonoBehaviour, IVisualFactory, IPriority
        where T : Message
    {
        [SerializeField]
        protected string m_Topic;

        public virtual void Start()
        {
            if (m_Topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<T>(this, Priority);
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
            if (!AssertMessageType(message, meta)) return null;
            return new TextureVisual<T>((T)message, meta, this, CreateTexture((T)message));
        }

        public abstract Texture2D CreateTexture(T message);

        public virtual Action CreateGUI(T message, MessageMetadata meta, Texture2D tex)
        {
            return MessageVisualizations.CreateDefaultGUI(message, meta);
        }

        public bool AssertMessageType(Message message, MessageMetadata meta)
        {
            if (!(message is T))
            {
                Debug.LogError($"{GetType()}, visualFactory for topic \"{meta.Topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(T)}).");
                return false;
            }

            return true;
        }
    }
}
