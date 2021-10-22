using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public abstract class GuiVisualizer<T> : BaseVisualFactory<T>, IPriority
        where T : Message
    {
        protected override IVisual CreateVisual(string topic)
        {
            return new GuiVisual(topic, this);
        }

        public override bool CanShowDrawing => false;

        public virtual Action CreateGUI(T message, MessageMetadata meta)
        {
            return VisualizationUtils.CreateDefaultGUI(message, meta);
        }

        public class GuiVisual : IVisual
        {
            string m_Topic;
            GuiVisualizer<T> m_Factory;

            Action m_GUIAction;

            public GuiVisual(string topic, GuiVisualizer<T> factory)
            {
                m_Topic = topic;
                m_Factory = factory;
                ListenForMessages(topic, AddMessage);
            }

            public void AddMessage(Message message)
            {
                if (!VisualizationUtils.AssertMessageType<T>(message, m_Topic))
                    return;

                this.message = (T)message;
                this.meta = new MessageMetadata(m_Topic, Time.time, DateTime.Now);
                m_GUIAction = null;
            }

            public T message { get; private set; }
            public MessageMetadata meta { get; private set; }

            public bool hasDrawing => false;
            public bool hasAction => m_GUIAction != null;

            public void OnGUI()
            {
                if (message == null)
                {
                    GUILayout.Label("Waiting for message...");
                    return;
                }

                if (m_GUIAction == null)
                {
                    m_GUIAction = m_Factory.CreateGUI(message, meta);
                }
                m_GUIAction();
            }

            public bool IsDrawingEnabled => false;
            public void SetDrawingEnabled(bool enabled) { }
            public void Redraw() { }
        }
    }
}
