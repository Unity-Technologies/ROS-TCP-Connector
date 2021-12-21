using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public abstract class HistoryDrawingVisualizer<T> : BaseVisualFactory<T>
        where T : Message
    {
        [SerializeField]
        int m_HistoryLength;
        public int HistoryLength { get => m_HistoryLength; set => m_HistoryLength = value; }

        public override bool CanShowDrawing => true;

        protected override IVisual CreateVisual(string topic)
        {
            return new HistoryDrawingVisual(topic, this, m_HistoryLength);
        }

        public virtual void Draw(Drawing3d drawing, IEnumerable<Tuple<T, MessageMetadata>> messages)
        {
            Draw(drawing, messages.Select(Tuple => Tuple.Item1));
        }

        public virtual void Draw(Drawing3d drawing, IEnumerable<T> messages) { }

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
            return VisualizationUtils.CreateDefaultGUI(message, meta);
        }

        public class HistoryDrawingVisual : IVisual
        {
            string m_Topic;
            Queue<Tuple<T, MessageMetadata>> m_History = new Queue<Tuple<T, MessageMetadata>>();

            Drawing3d m_BasicDrawing;
            Action m_GUIAction;
            HistoryDrawingVisualizer<T> m_Factory;
            int m_HistoryLength;
            bool m_IsDrawingEnabled;
            public bool IsDrawingEnabled => m_IsDrawingEnabled;
            float m_LastDrawingFrameTime = -1;

            public HistoryDrawingVisual(string topic, HistoryDrawingVisualizer<T> factory, int historyLength)
            {
                m_Topic = topic;
                m_Factory = factory;
                m_HistoryLength = historyLength;
                ListenForMessages(topic, AddMessage);
            }

            public void AddMessage(Message message)
            {
                if (!VisualizationUtils.AssertMessageType<T>(message, m_Topic))
                    return;

                m_History.Enqueue(new Tuple<T, MessageMetadata>(
                    (T)message,
                    new MessageMetadata(m_Topic, Time.time, DateTime.Now)
                ));
                if (m_History.Count > m_HistoryLength)
                    m_History.Dequeue();
                m_GUIAction = null;

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
                    m_GUIAction = m_Factory.CreateGUI(m_History);
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

            public void Redraw()
            {
                if (m_BasicDrawing == null)
                {
                    m_BasicDrawing = Drawing3dManager.CreateDrawing();
                }
                else
                {
                    m_BasicDrawing.Clear();
                }

                m_Factory.Draw(m_BasicDrawing, m_History);
            }
        }
    }
}
