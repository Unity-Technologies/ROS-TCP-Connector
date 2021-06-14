using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class BasicVisualization : IMessageVisualization
    {
        BasicDrawing m_BasicDrawing;
        Action m_GUIAction;

        public BasicVisualization(Message newMessage, MessageMetadata newMeta, Action action, BasicDrawing drawing)
        {
            message = newMessage;
            meta = newMeta;
            m_GUIAction = action;
            m_BasicDrawing = drawing;
        }

        public Message message { get; }
        public MessageMetadata meta { get; }

        public bool hasDrawing
        {
            get => m_BasicDrawing != null;
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
            m_BasicDrawing.Destroy();
            m_BasicDrawing = null;
        }
    }
}
