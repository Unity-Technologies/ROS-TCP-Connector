using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DrawingVisual<TargetMessageType> : IVisual
        where TargetMessageType : Message
    {
        public TargetMessageType message { get; }
        Message IVisual.message
        {
            get => message;
        }
        public MessageMetadata meta { get; }
        BasicDrawing m_BasicDrawing;
        DrawingVisualFactory<TargetMessageType> m_Factory;
        
        public DrawingVisual(TargetMessageType newMessage, MessageMetadata newMeta, DrawingVisualFactory<TargetMessageType> factory)
        {
            message = newMessage;
            meta = newMeta;
            m_Factory = factory;
        }
        
        public bool hasDrawing => m_BasicDrawing != null;
        public bool hasAction => false;
        
        public void DeleteDrawing()
        {
            if (m_BasicDrawing != null)
            {
                m_BasicDrawing.Destroy();
            }

            m_BasicDrawing = null;
        }

        public void OnGUI() { }

        public void CreateDrawing()
        {
            if (m_BasicDrawing == null)
            {
                m_BasicDrawing = m_Factory.CreateDrawing(message, meta, null);
            }
        }

        public void Recycle(IVisual oldVisual)
        {
            if (oldVisual is DrawingVisual<TargetMessageType> v)
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
