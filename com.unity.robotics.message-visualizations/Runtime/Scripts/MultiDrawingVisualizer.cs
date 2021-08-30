using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class MultiDrawingVisualizer<T> : BaseVisualFactory<T>
        where T : Message
    {
        [SerializeField]
        int m_HistoryLength;
        public int HistoryLength { get => m_HistoryLength; set => m_HistoryLength = value; }

        public override bool CanShowDrawing => true;

        protected override IVisual CreateVisual(string topic)
        {
            return new MultiDrawingVisual(topic, this, m_HistoryLength);
        }

        public virtual void Draw(BasicDrawing drawing, IEnumerable<Tuple<T, MessageMetadata>> messages) { }

        public virtual Action CreateGUI(IEnumerable<Tuple<T, MessageMetadata>> messages)
        {
            List<Action> actions = new List<Action>();
            foreach ((T message, MessageMetadata meta) in messages)
            {
                actions.Add(CreateGUI(message, meta));
            }
            return () => actions.ForEach(a => a());
        }

        public virtual Action CreateGUI(T message, MessageMetadata meta)
        {
            return MessageVisualizationUtils.CreateDefaultGUI(message, meta);
        }

        public class MultiDrawingVisual : IVisual
        {
            string m_Topic;
            Queue<Tuple<T, MessageMetadata>> messages = new Queue<Tuple<T, MessageMetadata>>();

            BasicDrawing m_BasicDrawing;
            Action m_GUIAction;
            MultiDrawingVisualizer<T> m_Factory;
            int m_HistoryLength;
            bool m_IsDrawingEnabled;
            float m_LastDrawingFrameTime = -1;

            public MultiDrawingVisual(string topic, MultiDrawingVisualizer<T> factory, int historyLength)
            {
                m_Topic = topic;
                m_Factory = factory;
                m_HistoryLength = historyLength;

                ROSConnection.GetOrCreateInstance().Subscribe<T>(m_Topic, AddMessage);
            }

            public void AddMessage(Message message)
            {
                if (!MessageVisualizationUtils.AssertMessageType<T>(message, m_Topic))
                    return;

                messages.Enqueue(new Tuple<T, MessageMetadata>(
                    (T)message,
                    new MessageMetadata(m_Topic, Time.time, DateTime.Now)
                ));
                if (messages.Count > m_HistoryLength)
                    messages.Dequeue();
                m_GUIAction = null;

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
}
