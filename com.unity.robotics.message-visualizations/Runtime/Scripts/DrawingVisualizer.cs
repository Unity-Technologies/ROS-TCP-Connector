using RosMessageTypes.Std;
using System;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class DrawingVisualizer<T> : BaseVisualFactory<T>
        where T : Message
    {
        public override bool CanShowDrawing => true;

        protected override IVisual CreateVisual(string topic)
        {
            return new DrawingVisual(topic, this);
        }

        public Color SelectColor(Color userColor, MessageMetadata meta)
        {
            return MessageVisualizationUtils.SelectColor(userColor, meta);
        }

        public string SelectLabel(string userLabel, MessageMetadata meta)
        {
            return MessageVisualizationUtils.SelectLabel(userLabel, meta);
        }

        public virtual void Draw(DrawingVisual drawing, T message, MessageMetadata meta)
        {
            Draw(drawing.BasicDrawing, message, meta);
        }

        public virtual void Draw(BasicDrawing drawing, T message, MessageMetadata meta) { }

        public virtual Action CreateGUI(T message, MessageMetadata meta)
        {
            return MessageVisualizationUtils.CreateDefaultGUI(message, meta);
        }

        public class DrawingVisual : IVisual
        {
            public T message { get; private set; }
            public MessageMetadata meta { get; private set; }

            BasicDrawing m_BasicDrawing;
            public BasicDrawing BasicDrawing => m_BasicDrawing;
            Action m_GUIAction;
            DrawingVisualizer<T> m_Factory;
            string m_Topic;
            bool m_IsDrawingEnabled;
            float m_LastDrawingFrameTime = -1;

            public DrawingVisual(string topic, DrawingVisualizer<T> factory)
            {
                m_Topic = topic;
                m_Factory = factory;

                ROSConnection.GetOrCreateInstance().Subscribe<T>(topic, AddMessage);
            }

            public virtual void AddMessage(Message message)
            {
                MessageMetadata meta = new MessageMetadata(m_Topic, Time.time, DateTime.Now);

                if (!MessageVisualizationUtils.AssertMessageType<T>(message, m_Topic))
                    return;

                this.message = (T)message;
                this.meta = meta;
                m_GUIAction = null;

                // If messages are coming in faster than 1 per frame, we only update the drawing once per frame
                if (m_IsDrawingEnabled && Time.time > m_LastDrawingFrameTime)
                {
                    CreateDrawing();
                }

                m_LastDrawingFrameTime = Time.time;
            }

            public void SetDrawingEnabled(bool enabled)
            {
                if (m_IsDrawingEnabled == enabled)
                    return;

                if (!enabled && m_BasicDrawing != null)
                {
                    m_BasicDrawing.Clear();
                }
                m_IsDrawingEnabled = enabled;
            }

            public bool hasDrawing => m_BasicDrawing != null;

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

            public void DeleteDrawing()
            {
                if (m_BasicDrawing != null)
                {
                    m_BasicDrawing.Destroy();
                }

                m_BasicDrawing = null;
            }

            public void CreateDrawing()
            {
                if (m_BasicDrawing == null)
                {
                    m_BasicDrawing = BasicDrawingManager.CreateDrawing();
                }
                else
                {
                    m_BasicDrawing.Clear();
                }

                m_Factory.Draw(this, message, meta);
            }
        }
    }
}
