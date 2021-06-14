using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Unity.Robotics.MessageVisualizers
{
    public class BasicTextureVisualization : ITextureMessageVisualization
    {
        Action m_GUIAction;
        Texture2D m_Texture2D;

        public BasicTextureVisualization(Message newMessage, MessageMetadata newMeta, Action action, Texture2D tex)
        {
            message = newMessage;
            meta = newMeta;
            m_GUIAction = action;
            m_Texture2D = tex;
        }

        public Message message { get; }
        public MessageMetadata meta { get; }

        public bool hasDrawing
        {
            get => false;
            set => Delete();
        }

        public bool hasAction
        {
            get => m_GUIAction != null;
            set => m_GUIAction = null;
        }

        public void OnGUI()
        {
            if (m_GUIAction != null) m_GUIAction();
        }

        public void Delete()
        {
            Object.Destroy(m_Texture2D);
        }

        public Texture2D GetTexture()
        {
            return m_Texture2D;
        }

        public void SetTexture(Texture2D tex)
        {
            m_Texture2D = tex;
        }
    }
}
