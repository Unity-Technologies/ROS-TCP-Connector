using RosMessageTypes.Std;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class DrawingVisualizer<T> : MonoBehaviour, IVisualFactory, IPriority
        where T : Message
    {
        [SerializeField]
        string m_Topic;
        public string Topic { get => m_Topic; set => m_Topic = value; }

        public virtual void Start()
        {
            if (string.IsNullOrEmpty(m_Topic))
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<T>(this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(m_Topic, this, Priority);
            }
        }

        public int Priority { get; set; }

        public bool CanShowDrawing => true;

        public virtual IVisual CreateVisual()
        {
            return new DrawingVisual(this);
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

            public DrawingVisual(DrawingVisualizer<T> factory)
            {
                m_Factory = factory;
            }

            public virtual void AddMessage(Message message, MessageMetadata meta)
            {
                if (!MessageVisualizationUtils.AssertMessageType<T>(message, meta))
                    return;

                this.message = (T)message;
                this.meta = meta;
                m_GUIAction = null;
            }

            public bool hasDrawing => m_BasicDrawing != null;
            public bool hasAction => m_GUIAction != null;

            public void OnGUI()
            {
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
