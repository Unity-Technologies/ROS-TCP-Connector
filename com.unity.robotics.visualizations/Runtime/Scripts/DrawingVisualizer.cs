using RosMessageTypes.Std;
using System;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
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
            return VisualizationUtils.SelectColor(userColor, meta);
        }

        public string SelectLabel(string userLabel, MessageMetadata meta)
        {
            return VisualizationUtils.SelectLabel(userLabel, meta);
        }

        public virtual void Draw(DrawingVisual drawing, T message, MessageMetadata meta)
        {
            Draw(drawing.Drawing, message, meta);
        }

        public virtual void Draw(Drawing3d drawing, T message, MessageMetadata meta) { }

        public virtual Action CreateGUI(T message, MessageMetadata meta)
        {
            return VisualizationUtils.CreateDefaultGUI(message, meta);
        }

        public class DrawingVisual : IVisual
        {
            public T message { get; private set; }
            public MessageMetadata meta { get; private set; }

            Drawing3d m_Drawing;
            public Drawing3d Drawing => m_Drawing;
            Action m_GUIAction;
            DrawingVisualizer<T> m_Factory;
            string m_Topic;
            bool m_IsDrawingEnabled;
            public bool IsDrawingEnabled => m_IsDrawingEnabled;
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

                if (!VisualizationUtils.AssertMessageType<T>(message, m_Topic))
                    return;

                this.message = (T)message;
                this.meta = meta;
                m_GUIAction = null;

                // If messages are coming in faster than 1 per frame, we only update the drawing once per frame
                if (m_IsDrawingEnabled && Time.time > m_LastDrawingFrameTime)
                {
                    Redraw();
                }

                m_LastDrawingFrameTime = Time.time;
            }

            public void SetDrawingEnabled(bool enabled)
            {
                if (m_IsDrawingEnabled == enabled)
                    return;

                if (!enabled && m_Drawing != null)
                {
                    m_Drawing.Clear();
                }
                m_IsDrawingEnabled = enabled;
            }

            public bool hasDrawing => m_Drawing != null;

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
                if (m_Drawing != null)
                {
                    m_Drawing.Destroy();
                }

                m_Drawing = null;
            }

            public void Redraw()
            {
                if (m_Drawing == null)
                {
                    m_Drawing = Drawing3dManager.CreateDrawing();
                }
                else
                {
                    m_Drawing.Clear();
                }

                m_Factory.Draw(this, message, meta);
            }
        }
    }
}
