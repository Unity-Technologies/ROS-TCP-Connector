using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class GuiVisual<T> : IVisual
        where T : Message
    {
        GuiVisualFactory<T> m_Factory;

        Action m_GUIAction;

        public GuiVisual(GuiVisualFactory<T> factory)
        {
            m_Factory = factory;
        }

        public void NewMessage(Message message, MessageMetadata meta)
        {
            if (!MessageVisualizationUtils.AssertMessageType<T>(message, meta))
                return;

            this.message = (T)message;
            this.meta = meta;
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
                m_GUIAction = m_Factory.CreateGUI(message, meta);
            }
            m_GUIAction();
        }

        public void DeleteDrawing() { }
        public void CreateDrawing() { }
    }
}
