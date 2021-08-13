using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class MultiDrawingVisual<T> : IVisual
        where T : Message
    {
        Queue<Tuple<T, MessageMetadata>> messages = new Queue<Tuple<T, MessageMetadata>>();

        BasicDrawing m_BasicDrawing;
        Action m_GUIAction;
        MultiDrawingVisualFactory<T> m_Factory;
        int m_HistoryLength;

        public MultiDrawingVisual(MultiDrawingVisualFactory<T> factory, int historyLength)
        {
            m_Factory = factory;
            m_HistoryLength = historyLength;
        }

        public void NewMessage(Message message, MessageMetadata meta)
        {
            if (!MessageVisualizationUtils.AssertMessageType<T>(message, meta))
                return;

            messages.Enqueue(new Tuple<T, MessageMetadata>((T)message, meta));
            if (messages.Count > m_HistoryLength)
                messages.Dequeue();
            m_GUIAction = null;
        }

        public bool hasDrawing => m_BasicDrawing != null;
        public bool hasAction => m_GUIAction != null;

        public void OnGUI()
        {
            if (m_GUIAction == null)
            {
                m_GUIAction = m_Factory.CreateGUI(messages);
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

            m_Factory.Draw(m_BasicDrawing, messages);
        }
    }
}
