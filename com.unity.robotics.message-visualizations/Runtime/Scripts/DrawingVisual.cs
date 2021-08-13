using RosMessageTypes.Std;
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

    public class DrawingVisual<TFactory, TMessage> : IVisual
        where TFactory : DrawingVisualFactory<TMessage>
        where TMessage : Message
    {
        public TMessage message { get; private set; }
        public MessageMetadata meta { get; private set; }

        BasicDrawing m_BasicDrawing;
        Action m_GUIAction;
        TFactory m_Factory;
        TFTrackingType m_TFTrackingType;
        HeaderMsg m_HeaderMsg;

        public DrawingVisual(TFactory factory, TFTrackingType tfTrackingType = TFTrackingType.None, HeaderMsg headerMsg = null)
        {
            m_Factory = factory;
            m_TFTrackingType = tfTrackingType;
        }

        public void NewMessage(Message message, MessageMetadata meta)
        {
            if (!AssertMessageType(message, meta))
                return;

            m_HeaderMsg = m_Factory.GetHeader(message);
            this.message = (TMessage)message;
            this.meta = meta;
        }

        public bool AssertMessageType(Message message, MessageMetadata meta)
        {
            if (!(message is TMessage))
            {
                Debug.LogError($"{GetType()}, visualFactory for topic \"{meta.Topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(TMessage)}).");
                return false;
            }

            return true;
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

            m_Factory.Draw(m_BasicDrawing, message, meta);
        }
    }
}
