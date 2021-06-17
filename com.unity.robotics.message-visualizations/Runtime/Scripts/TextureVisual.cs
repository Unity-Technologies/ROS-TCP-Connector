using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class TextureVisual<TargetMessageType> : ITextureVisual
        where TargetMessageType : Message
    {
        TextureVisualFactory<TargetMessageType> m_Factory;

        Action m_GUIAction;
        Texture2D m_Texture2D;

        public TextureVisual(TargetMessageType newMessage, MessageMetadata newMeta, TextureVisualFactory<TargetMessageType> factory, Texture2D tex)
        {
            message = newMessage;
            meta = newMeta;
            m_Factory = factory;
            m_Texture2D = tex;
        }

        TargetMessageType message { get; }

        Message IVisual.message => message;

        public MessageMetadata meta { get; }

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
        public void Recycle(IVisual oldVisual) { }
    }
}
