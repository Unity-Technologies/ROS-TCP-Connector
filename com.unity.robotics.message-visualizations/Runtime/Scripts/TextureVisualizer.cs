using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class TextureVisualizer<T> : BaseVisualFactory<T>
        where T : Message
    {
        public override bool CanShowDrawing => false;

        protected override IVisual CreateVisual(string topic)
        {
            return new TextureVisual(topic, this);
        }

        public abstract Texture2D CreateTexture(T message);

        public virtual Action CreateGUI(T message, MessageMetadata meta, Texture2D tex)
        {
            return MessageVisualizationUtils.CreateDefaultGUI(message, meta);
        }

        public class TextureVisual : ITextureVisual
        {
            string m_Topic;
            TextureVisualizer<T> m_Factory;

            Action m_GUIAction;
            Texture2D m_Texture2D;
            List<Action<Texture2D>> m_OnChangeCallbacks = new List<Action<Texture2D>>();

            public void ListenForTextureChange(Action<Texture2D> callback)
            {
                m_OnChangeCallbacks.Add(callback);
            }

            public TextureVisual(string topic, TextureVisualizer<T> factory)
            {
                m_Topic = topic;
                m_Factory = factory;

                ROSConnection.GetOrCreateInstance().Subscribe<T>(topic, AddMessage);
            }

            public void AddMessage(Message message)
            {
                if (!MessageVisualizationUtils.AssertMessageType<T>(message, m_Topic))
                    return;

                this.message = (T)message;
                this.meta = meta;
                m_Texture2D = null;
                m_GUIAction = null;

                // notify anyone who requested updates when the texture changes
                foreach (Action<Texture2D> callback in m_OnChangeCallbacks)
                {
                    callback(GetTexture());
                }
            }

            public T message { get; private set; }

            public MessageMetadata meta { get; private set; }

            public void OnGUI()
            {
                if (message == null)
                {
                    GUILayout.Label("Waiting for message...");
                    return;
                }

                if (m_GUIAction == null)
                {
                    m_GUIAction = m_Factory.CreateGUI(message, meta, GetTexture());
                }
                m_GUIAction();
            }

            public Texture2D GetTexture()
            {
                if (m_Texture2D == null && message != null)
                {
                    m_Texture2D = m_Factory.CreateTexture(message);
                }
                return m_Texture2D;
            }

            public void SetDrawingEnabled(bool enabled) { }

            public void CreateDrawing() { }
        }
    }
}
