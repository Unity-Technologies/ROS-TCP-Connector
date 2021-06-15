using System;
using System.Security.Cryptography;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class BasicVisualization<TargetMessageType> : IVisual
        where TargetMessageType : Message
    {
        public TargetMessageType message { get; }
        Message IVisual.message
        {
            get => message;
        }
        public MessageMetadata meta { get; }
        
        BasicDrawing m_BasicDrawing;
        Action m_GUIAction;
        BasicVisualFactory<TargetMessageType> m_Factory;

        public BasicVisualization(TargetMessageType newMessage, MessageMetadata newMeta, BasicVisualFactory<TargetMessageType> factory)
        {
            message = newMessage;
            meta = newMeta;
            m_Factory = factory;
        }

        public bool hasDrawing
        {
            get => m_BasicDrawing != null;
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

        public void DeleteDrawing()
        {
            if (m_BasicDrawing != null)
            {
                Debug.Log($"Calling destroy on {meta.Topic} drawing");
                m_BasicDrawing.Destroy();
            }
            m_BasicDrawing = null;
        }
        
        public void CreateDrawing()
        {
            if (m_BasicDrawing != null)
            {
                DeleteDrawing();
            }
            m_BasicDrawing ??= m_Factory.CreateDrawing(message, meta, null);
        }
    }
}
