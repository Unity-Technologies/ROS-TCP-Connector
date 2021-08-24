using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class GuiVisualizer<T> : BaseVisualFactory<T>, IPriority
        where T : Message
    {
        protected override IVisual CreateVisual()
        {
            return new GuiVisual(this);
        }

        public override bool CanShowDrawing => false;

        public virtual Action CreateGUI(T message, MessageMetadata meta)
        {
            return MessageVisualizationUtils.CreateDefaultGUI(message, meta);
        }

        public class GuiVisual : IVisual
        {
            GuiVisualizer<T> m_Factory;

            Action m_GUIAction;

            public GuiVisual(GuiVisualizer<T> factory)
            {
                m_Factory = factory;
            }

            public void AddMessage(Message message, MessageMetadata meta)
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
}
