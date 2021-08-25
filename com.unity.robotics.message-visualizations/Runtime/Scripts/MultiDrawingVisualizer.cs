using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
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

        protected override IVisual CreateVisual()
        {
            return new Visual(this, m_HistoryLength);
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

        public class Visual : IVisual
        {
            Queue<Tuple<T, MessageMetadata>> messages = new Queue<Tuple<T, MessageMetadata>>();

            BasicDrawing m_BasicDrawing;
            Action m_GUIAction;
            MultiDrawingVisualizer<T> m_Factory;
            int m_HistoryLength;

            public Visual(MultiDrawingVisualizer<T> factory, int historyLength)
            {
                m_Factory = factory;
                m_HistoryLength = historyLength;
            }

            public void AddMessage(Message message, MessageMetadata meta)
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
}
