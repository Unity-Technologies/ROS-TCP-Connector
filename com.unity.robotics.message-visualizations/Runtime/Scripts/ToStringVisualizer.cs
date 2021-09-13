using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    class ToStringVisualizer : IVisualFactory
    {
        public string Name => "ToString";
        public string ID => "ToString";

        Dictionary<string, IVisual> m_Visuals = new Dictionary<string, IVisual>();

        public IVisual GetOrCreateVisual(string topic)
        {
            IVisual visual;
            if (m_Visuals.TryGetValue(topic, out visual))
                return visual;

            visual = new ToStringVisual(topic);
            m_Visuals.Add(topic, visual);
            return visual;
        }

        // The ToString visualizer is the default visualizer. If you're trying to register it, something has gone extremely wrong...
        public void Register(int priority) { throw new NotImplementedException(); }

        public bool CanShowDrawing => false;

        class ToStringVisual : IVisual
        {
            public Message message { get; private set; }

            public ToStringVisual(string topic)
            {
                ROSConnection ros = ROSConnection.GetOrCreateInstance();
                RosTopicState state = ros.GetTopic(topic);
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
            public void SetDrawingEnabled(bool enabled) { }
            public void Redraw() { }
        }
    }
}
