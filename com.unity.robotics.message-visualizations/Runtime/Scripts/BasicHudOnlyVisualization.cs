using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.MessageVisualizers
{
    public class BasicHudOnlyVisualization<TargetMessageType> : IVisual
        where TargetMessageType : Message
    {
        BasicHudOnlyVisualFactory<TargetMessageType> m_Factory;

        Action m_GUIAction;

        public BasicHudOnlyVisualization(TargetMessageType newMessage, MessageMetadata newMeta, BasicHudOnlyVisualFactory<TargetMessageType> factory)
        {
            message = newMessage;
            meta = newMeta;
            m_Factory = factory;
        }

        public TargetMessageType message { get; }

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
            m_GUIAction ??= m_Factory.CreateGUI(message, meta);
            m_GUIAction();
        }

        public void CreateDrawing() { }

        public void DeleteDrawing() { }
    }
}
