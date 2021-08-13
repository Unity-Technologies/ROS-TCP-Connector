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
            this.message = (T)message;
            this.meta = meta;
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

        public T message { get; private set; }
        public MessageMetadata meta { get; private set; }

        //        Message IVisual.message => message;

        //        public MessageMetadata meta { get; }

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
