using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.MessageVisualizers
{
    public class GuiVisual<TargetMessageType> : IVisual
        where TargetMessageType : Message
    {
        GuiVisualFactory<TargetMessageType> m_Factory;

        Action m_GUIAction;

        public GuiVisual(TargetMessageType newMessage, MessageMetadata newMeta, GuiVisualFactory<TargetMessageType> factory)
        {
            message = newMessage;
            meta = newMeta;
            m_Factory = factory;
        }

        public TargetMessageType message { get; }

        Message IVisual.message => message;

        public MessageMetadata meta { get; }

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
        public void Recycle(IVisual oldVisual) { }
    }
}
