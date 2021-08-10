using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class VisualizationTopicState
    {
        RosTopicState m_BaseState;
        public string Topic => m_BaseState.Topic;
        public string RosMessageName => m_BaseState.RosMessageName;

        IVisualFactory m_VisualizerCached;
        bool m_NoVisualizerAvailable;
        bool m_SentSubscriberRegistration;

        public IVisualFactory GetVisualizer()
        {
            if (m_VisualizerCached == null && !m_NoVisualizerAvailable)
            {
                m_VisualizerCached = VisualFactoryRegistry.GetVisualizer(m_BaseState.Topic, m_BaseState.RosMessageName);
                if (m_VisualizerCached == null)
                    m_NoVisualizerAvailable = true;
            }
            return m_VisualizerCached;
        }

        IVisual m_Visual;
        public IVisual Visual => m_Visual;
        HudWindow m_VisualWindow;
        bool m_IsVisualizingUI;
        public bool IsVisualizingUI => m_IsVisualizingUI;
        bool m_IsVisualizingDrawing;
        public bool IsVisualizingDrawing => m_IsVisualizingDrawing;
        float m_LastVisualFrameTime;
        VisualizationTopicState m_ServiceResponseTopic;

        public VisualizationTopicState(RosTopicState baseState)
        {
            m_BaseState = baseState;
            if (baseState.ServiceResponseTopic != null)
            {
                m_ServiceResponseTopic = new VisualizationTopicState(baseState.ServiceResponseTopic);
            }
        }

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

        internal VisualizationTopicState(SaveState save, ROSConnection connection)
        {
            m_BaseState = connection.AddTopic(save.Topic, save.RosMessageName);
            if (save.HasRect && save.Rect.width > 0 && save.Rect.height > 0)
                m_VisualWindow = new HudWindow(save.Topic, save.Rect);
            else
                m_VisualWindow = new HudWindow(save.Topic);

            m_IsVisualizingUI = save.ShowWindow;
            m_IsVisualizingDrawing = save.ShowDrawing;
        }

        public SaveState CreateSaveState()
        {
            if (!m_IsVisualizingUI && !m_IsVisualizingDrawing)
                return null;

            return new SaveState
            {
                Rect = m_VisualWindow != null ? m_VisualWindow.WindowRect : new Rect(0, 0, 0, 0),
                HasRect = m_VisualWindow != null,
                Topic = m_BaseState.Topic,
                RosMessageName = m_BaseState.RosMessageName,
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

        void UpdateVisual(Message message)
        {
            MessageMetadata meta = new MessageMetadata(m_BaseState.Topic, Time.time, DateTime.Now);
            IVisualFactory visualizer = GetVisualizer();
            if (visualizer == null)
            {
                // this should never be null!? We know how to deserialize this message so the default visualizer should at least be working.
                Debug.LogError($"Unexpected error: No visualizer for {m_BaseState.RosMessageName} - message type {message?.GetType()}");
                return;
            }

            IVisual newVisual = visualizer.CreateVisual(message, meta);
            newVisual.Recycle(m_Visual);
            m_Visual = newVisual;
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
            if (GUILayout.Button(new GUIContent(m_BaseState.Topic, m_BaseState.RosMessageName), GUI.skin.label, GUILayout.Width(240)))
            {
                if (!canShowWindow)
                {
                    Debug.LogError($"No message class registered for type {m_BaseState.RosMessageName}");
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

            if (m_BaseState.ServiceResponseTopic != null)
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

            if ((ui || drawing) && !m_SentSubscriberRegistration)
            {
                m_BaseState.AddSubscriber(OnMessageReceived);
                m_SentSubscriberRegistration = true;
            }

            m_IsVisualizingUI = ui;
            m_IsVisualizingDrawing = drawing;
        }

    }
}
