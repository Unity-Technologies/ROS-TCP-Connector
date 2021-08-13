using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class TextureVisual<T> : ITextureVisual
        where T : Message
    {
        TextureVisualFactory<T> m_Factory;

        Action m_GUIAction;
        Texture2D m_Texture2D;

        public TextureVisual(TextureVisualFactory<T> factory)
        {
            m_Factory = factory;
        }

        public void NewMessage(Message message, MessageMetadata meta)
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
