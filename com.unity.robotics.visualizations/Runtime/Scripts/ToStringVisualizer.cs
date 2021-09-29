using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    class ToStringVisualizer : IVisualFactory
    {
        public string Name => "ToString";
        public string ID => "ToString";
        MessageSubtopic m_Subtopic;

        public ToStringVisualizer(MessageSubtopic subtopic)
        {
            m_Subtopic = subtopic;
        }

        Dictionary<string, IVisual> m_Visuals = new Dictionary<string, IVisual>();

        public IVisual GetOrCreateVisual(string topic)
        {
            IVisual visual;
            if (m_Visuals.TryGetValue(topic, out visual))
                return visual;

            visual = new ToStringVisual(topic, m_Subtopic);
            m_Visuals.Add(topic, visual);
            return visual;
        }

        // The ToString visualizer is the default visualizer. If you're trying to register it, something has gone extremely wrong...
        public void Register(int priority) { throw new NotImplementedException(); }

        public bool CanShowDrawing => false;

        class ToStringVisual : IVisual
        {
            public Message message { get; private set; }

            public ToStringVisual(string topic, MessageSubtopic subtopic)
            {
                ROSConnection ros = ROSConnection.GetOrCreateInstance();
                RosTopicState state = ros.GetTopic(topic);
                if (subtopic == MessageSubtopic.Response)
                    state = state.ServiceResponseTopic;
                state.AddSubscriber(AddMessage);
            }

            public void AddMessage(Message newMessage)
            {
                message = newMessage;
            }

            public void OnGUI()
            {
                if (message == null)
                {
                    GUILayout.Label("Waiting for message...");
                    return;
                }

                string text = message.ToString();
                GUILayout.Label(text);
            }

            public bool IsDrawingEnabled => false;
            public void SetDrawingEnabled(bool enabled) { }
            public void Redraw() { }
        }
    }
}
