using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    // Represents a single line in the VisualizationTopicsTab
    // and saves and loads the options for that line, plus the associated hud windows etc.
    public class VisualizationTopicsTabEntry
    {
        RosTopicState m_TopicState;
        public string Topic => m_TopicState.Topic;
        public string RosMessageName => m_TopicState.RosMessageName;

        string m_CachedRosMessageName;
        IVisualFactory m_VisualizerCached;
        bool m_NoVisualizerAvailable;
        bool m_DidSubscribe;

        IVisual m_Visual;
        public IVisual Visual => m_Visual;
        HudWindow m_VisualWindow;
        bool m_IsVisualizingUI;
        public bool IsVisualizingUI => m_IsVisualizingUI;
        bool m_IsVisualizingDrawing;
        public bool IsVisualizingDrawing => m_IsVisualizingDrawing;
        float m_LastVisualFrameTime = -1;
        // a service topic is represented by two lines, one for the request and one for the response. m_ServiceResponseTopic is the response.
        VisualizationTopicsTabEntry m_ServiceResponseTopic;

        [Serializable]
        public class SaveState
        {
            public Rect Rect;
            public bool HasRect;
            public string Topic;
            public string RosMessageName;
            public bool ShowWindow;
            public bool ShowDrawing;
        }

        public VisualizationTopicsTabEntry(RosTopicState baseState)
        {
            m_TopicState = baseState;
            if (baseState.ServiceResponseTopic != null)
            {
                m_ServiceResponseTopic = new VisualizationTopicsTabEntry(baseState.ServiceResponseTopic);
            }
        }

        internal VisualizationTopicsTabEntry(SaveState save, RosTopicState topicState)
        {
            m_TopicState = topicState;
            if (save.HasRect && save.Rect.width > 0 && save.Rect.height > 0)
                m_VisualWindow = new HudWindow(save.Topic, save.Rect);
            else
                m_VisualWindow = new HudWindow(save.Topic);

            SetVisualizing(save.ShowWindow, save.ShowDrawing);
        }

        public SaveState CreateSaveState()
        {
            if (!m_IsVisualizingUI && !m_IsVisualizingDrawing)
                return null;

            return new SaveState
            {
                Rect = m_VisualWindow != null ? m_VisualWindow.WindowRect : new Rect(0, 0, 0, 0),
                HasRect = m_VisualWindow != null,
                Topic = m_TopicState.Topic,
                RosMessageName = m_TopicState.RosMessageName,
                ShowWindow = m_IsVisualizingUI,
                ShowDrawing = m_IsVisualizingDrawing,
            };
        }

        public void OnMessageSent(Message message)
        {
            if (Time.time > m_LastVisualFrameTime && (m_IsVisualizingUI || m_IsVisualizingDrawing))
            {
                UpdateVisual(message);
            }
        }

        public void OnMessageReceived(Message message)
        {
            if (Time.time > m_LastVisualFrameTime && (m_IsVisualizingUI || m_IsVisualizingDrawing))
            {
                UpdateVisual(message);
            }
        }

        public IVisualFactory GetVisualizer()
        {
            if (m_CachedRosMessageName != RosMessageName)
            {
                // if the topic has changed, discard our cached data
                m_VisualizerCached = null;
                m_NoVisualizerAvailable = false;
            }
            if (m_VisualizerCached == null && !m_NoVisualizerAvailable)
            {
                m_VisualizerCached = VisualFactoryRegistry.GetVisualizer(m_TopicState.Topic, m_TopicState.RosMessageName);
                m_CachedRosMessageName = RosMessageName;
                if (m_VisualizerCached == null)
                    m_NoVisualizerAvailable = true;
            }
            return m_VisualizerCached;
        }

        void UpdateVisual(Message message)
        {
            MessageMetadata meta = new MessageMetadata(m_TopicState.Topic, Time.time, DateTime.Now);

            if (m_Visual == null)
            {
                IVisualFactory visualizer = GetVisualizer();
                if (visualizer == null)
                {
                    // this should never be null!? Clearly we know how to deserialize this message, so the default visualizer should at least be working.
                    Debug.LogError($"Unexpected error: No visualizer for {m_TopicState.RosMessageName} - message type {message?.GetType()}");
                    return;
                }
                m_Visual = visualizer.GetOrCreateVisual(m_TopicState.Topic);
            }
            m_Visual.AddMessage(message, meta);
            m_LastVisualFrameTime = Time.time;

            if (m_IsVisualizingDrawing)
                m_Visual.CreateDrawing();

            if (m_VisualWindow != null)
                m_VisualWindow.SetOnGUI(m_Visual.OnGUI);
        }

        public void DrawGUILine()
        {
            bool showWindow = IsVisualizingUI;
            bool showDrawing = IsVisualizingDrawing;

            IVisualFactory visualizer = GetVisualizer();
            bool canShowWindow = visualizer != null;
            bool canShowDrawing = visualizer != null ? visualizer.CanShowDrawing : false;

            var hasWindow = showWindow;
            var hasDrawing = showDrawing;

            GUILayout.BeginHorizontal();
            if (hasWindow || canShowWindow)
                showWindow = GUILayout.Toggle(showWindow, "", GUILayout.Width(15));
            else
                GUILayout.Label("", GUILayout.Width(15));

            if (hasDrawing || canShowDrawing)
                showDrawing = GUILayout.Toggle(showDrawing, "", GUILayout.Width(15));
            else
                GUILayout.Label("", GUILayout.Width(15));

            var baseColor = GUI.color;
            GUI.color = canShowWindow ? baseColor : Color.grey;
            if (GUILayout.Button(new GUIContent(m_TopicState.Topic, m_TopicState.RosMessageName), GUI.skin.label, GUILayout.Width(240)))
            {
                if (!canShowWindow)
                {
                    Debug.LogError($"No message class registered for type {m_TopicState.RosMessageName}");
                }
                else if (!canShowDrawing)
                {
                    showWindow = !showWindow;
                }
                else
                {
                    var toggleOn = !showWindow || !showDrawing;
                    showWindow = toggleOn;
                    showDrawing = toggleOn;
                }
            }

            GUI.color = baseColor;
            GUILayout.EndHorizontal();

            if (showDrawing != m_IsVisualizingDrawing || showWindow != m_IsVisualizingUI)
            {
                SetVisualizing(showWindow, showDrawing);
            }

            if (m_TopicState.ServiceResponseTopic != null)
            {
                m_ServiceResponseTopic.DrawGUILine();
            }
        }

        void DefaultWindowContents()
        {
            GUILayout.Label("Waiting for message...");
        }

        public void SetVisualizing(bool ui, bool drawing)
        {
            if (m_VisualWindow != null)
            {
                m_VisualWindow.SetActive(ui);
            }
            else if (ui)
            {
                m_VisualWindow = new HudWindow(Topic);
                m_VisualWindow.SetOnGUI(DefaultWindowContents);
                HudPanel.AddWindow(m_VisualWindow);
            }

            if (m_Visual != null)
            {
                m_Visual.DeleteDrawing();

                if (drawing)
                    m_Visual.CreateDrawing();

                if (m_VisualWindow != null)
                    m_VisualWindow.SetOnGUI(m_Visual.OnGUI);
            }

            if ((ui || drawing) && !m_DidSubscribe)
            {
                m_TopicState.AddSubscriber(OnMessageReceived);
                m_DidSubscribe = true;
            }

            m_IsVisualizingUI = ui;
            m_IsVisualizingDrawing = drawing;
        }
    }
}
