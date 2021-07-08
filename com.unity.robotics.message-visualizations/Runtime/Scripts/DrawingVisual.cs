using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public interface IVisualDrawer<T> where T : Message
    {
        void Draw(BasicDrawing drawing, T message, MessageMetadata meta);
        Action CreateGUI(T message, MessageMetadata meta);
    }

    public class DrawingVisual<T> : IVisual
        where T : Message
    {
        public T message { get; }
        Message IVisual.message
        {
            get => message;
        }
        public MessageMetadata meta { get; }

        BasicDrawing m_BasicDrawing;
        Action m_GUIAction;
        IVisualDrawer<T> m_Drawer;

        public DrawingVisual(T newMessage, MessageMetadata newMeta, IVisualDrawer<T> drawer)
        {
            message = newMessage;
            meta = newMeta;
            m_Drawer = drawer;
        }

        public bool hasDrawing => m_BasicDrawing != null;
        public bool hasAction => m_GUIAction != null;

        public void OnGUI()
        {
            if (m_GUIAction == null)
            {
                m_GUIAction = m_Drawer.CreateGUI(message, meta);
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

            m_Drawer.Draw(m_BasicDrawing, message, meta);
        }

        public void Recycle(IVisual oldVisual)
        {
            if (oldVisual is DrawingVisual<T> v)
            {
                m_BasicDrawing = v.m_BasicDrawing;
                v.m_BasicDrawing = null;
                if (m_BasicDrawing != null)
                {
                    m_BasicDrawing.Clear();
                }
            }
        }
    }
}
