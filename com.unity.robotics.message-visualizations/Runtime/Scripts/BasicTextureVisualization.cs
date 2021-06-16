using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class BasicTextureVisualization<TargetMessageType> : ITextureVisual
        where TargetMessageType : Message
    {
        BasicTextureVisualFactory<TargetMessageType> m_Factory;

        Action m_GUIAction;
        Texture2D m_Texture2D;

        public BasicTextureVisualization(TargetMessageType newMessage, MessageMetadata newMeta, BasicTextureVisualFactory<TargetMessageType> factory, Texture2D tex)
        {
            message = newMessage;
            meta = newMeta;
            m_Factory = factory;
            m_Texture2D = tex;
        }

        TargetMessageType message { get; }

        Message IVisual.message => message;

        public MessageMetadata meta { get; }

        public bool hasDrawing
        {
            get => false;
            set
            {
                if (!value) DeleteDrawing();
            }
        }

        public bool hasAction
        {
            get => m_GUIAction != null;
            set
            {
                if (!value) m_GUIAction = null;
            }
        }

        public void OnGUI()
        {
            m_GUIAction ??= m_Factory.CreateGUI(message, meta, GetTexture());
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

        public void CreateDrawing() { }

        public void DeleteDrawing() { }
    }
}
