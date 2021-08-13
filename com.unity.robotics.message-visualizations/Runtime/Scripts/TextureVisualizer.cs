using RosMessageTypes.Std;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class TextureVisualizer<T> : MonoBehaviour, IVisualFactory, IPriority
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

        public IVisual CreateVisual()
        {
            return new Visual(this);
        }

        public abstract Texture2D CreateTexture(T message);

        public virtual Action CreateGUI(T message, MessageMetadata meta, Texture2D tex)
        {
            return MessageVisualizationUtils.CreateDefaultGUI(message, meta);
        }

        public class Visual : ITextureVisual
        {
            TextureVisualizer<T> m_Factory;

            Action m_GUIAction;
            Texture2D m_Texture2D;

            public Visual(TextureVisualizer<T> factory)
            {
                m_Factory = factory;
            }

            public void AddMessage(Message message, MessageMetadata meta)
            {
                if (!MessageVisualizationUtils.AssertMessageType<T>(message, meta))
                    return;

                this.message = (T)message;
                this.meta = meta;
                m_Texture2D = null;
                m_GUIAction = null;
            }

            public T message { get; private set; }

            public MessageMetadata meta { get; private set; }

            public bool hasDrawing => false;
            public bool hasAction => m_GUIAction != null;

            public void OnGUI()
            {
                if (m_GUIAction == null)
                {
                    m_GUIAction = m_Factory.CreateGUI(message, meta, GetTexture());
                }
                m_GUIAction();
            }

            public Texture2D GetTexture()
            {
                if (m_Texture2D == null)
                {
                    m_Texture2D = m_Factory.CreateTexture(message);
                }
                return m_Texture2D;
            }

            public void DeleteDrawing() { }
            public void CreateDrawing() { }
        }

    }
}
