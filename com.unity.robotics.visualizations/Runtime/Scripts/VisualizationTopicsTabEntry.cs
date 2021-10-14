using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    // Represents a single line in the VisualizationTopicsTab
    // and saves and loads the options for that line, plus the associated hud windows etc.
    public class VisualizationTopicsTabEntry
    {
        RosTopicState m_TopicState;
        public RosTopicState TopicState => m_TopicState;
        public string Topic => m_TopicState.Topic;
        public MessageSubtopic Subtopic => m_TopicState.Subtopic;
        public string RosMessageName => m_TopicState.RosMessageName;
        public string Title => Topic + (Subtopic == MessageSubtopic.Response ? " (response)" : "");

        // a service topic is represented by two lines, one for the request and one for the response. m_ServiceResponseTopic is the response.
        VisualizationTopicsTabEntry m_ServiceResponseTopic;
        Texture2D m_Background;
        GUIStyle m_LineStyle;
        GUIStyle m_HoverStyle;

        bool m_IsVisualizingUI;
        public bool IsVisualizingUI => m_IsVisualizingUI;

        bool m_IsVisualizingDrawing;
        public bool IsVisualizingDrawing => m_IsVisualizingDrawing;

        string m_CachedRosMessageName;

        IVisualFactory m_VisualFactory;
        bool m_NoVisualFactoryAvailable;

        IVisual m_Visual;
        public IVisual Visual => m_Visual;
        HudWindow m_VisualWindow;
        bool m_IsHovering;

        public IVisualFactory GetVisualFactory()
        {
            if (m_CachedRosMessageName != RosMessageName)
            {
                // if the topic has changed, discard our cached data
                m_VisualFactory = null;
                m_NoVisualFactoryAvailable = false;
            }

            if (m_VisualFactory == null && !m_NoVisualFactoryAvailable)
            {
                SetVisualFactory(VisualFactoryRegistry.GetVisualFactory(Topic, RosMessageName, Subtopic));
            }
            return m_VisualFactory;
        }

        public void SetVisualFactory(IVisualFactory visualFactory)
        {
            if (m_Visual != null)
                m_Visual.SetDrawingEnabled(false);
            m_Visual = null;

            m_VisualFactory = visualFactory;
            m_CachedRosMessageName = RosMessageName;
            if (m_VisualFactory == null)
                m_NoVisualFactoryAvailable = true;

            SetVisualizing(m_IsVisualizingUI, m_IsVisualizingDrawing);
        }

        public bool CanShowWindow => GetVisualFactory() != null;
        public bool CanShowDrawing => GetVisualFactory() != null && m_VisualFactory.CanShowDrawing;

        public void DrawGUI()
        {
            DrawGUILine();

            if (m_ServiceResponseTopic != null)
            {
                m_ServiceResponseTopic.DrawGUILine();
            }
        }

        public void DrawGUILine()
        {
            bool showWindow = IsVisualizingUI;
            bool showDrawing = IsVisualizingDrawing;

            bool canShowWindow = CanShowWindow;
            bool canShowDrawing = CanShowDrawing;

            var hasWindow = showWindow;
            var hasDrawing = showDrawing;

            if (m_LineStyle == null)
                InitLineStyle();

            GUILayout.BeginHorizontal(m_IsHovering ? m_HoverStyle : m_LineStyle);
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

            GUILayout.Space(40);
            Rect space = GUILayoutUtility.GetLastRect();
            ROSConnection.DrawConnectionArrows(false,
                space.x + 5, space.y,
                Time.realtimeSinceStartup - TopicState.LastMessageReceivedRealtime,
                Time.realtimeSinceStartup - TopicState.LastMessageSentRealtime,
                TopicState.IsPublisher,
                TopicState.SentSubscriberRegistration,
                m_TopicState.Connection.HasConnectionError);

            if (GUILayout.Button(Title, m_LineStyle, GUILayout.Width(240)))
            {
                if (!canShowWindow)
                {
                    Debug.LogError($"No message class registered for type {RosMessageName}");
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

            Rect horizontalRect = GUILayoutUtility.GetLastRect();

#if UNITY_EDITOR
            Rect buttonRect = new Rect(horizontalRect.xMax - 20, horizontalRect.center.y - 10, 20, 20);
            if (GUI.Button(buttonRect, "\u2630"))
                ShowOptionsMenu(buttonRect);
#endif

            if (m_IsHovering)
            {
                string labelText = RosMessageName;
                Vector2 labelSize = GUI.skin.box.CalcSize(new GUIContent(labelText));

                GUI.Box(new Rect(horizontalRect.xMax - labelSize.x, horizontalRect.yMax - 10, labelSize.x, horizontalRect.height), labelText);
            }

            if (Event.current.type == EventType.Repaint)
                m_IsHovering = horizontalRect.Contains(Event.current.mousePosition);

            if (showDrawing != m_IsVisualizingDrawing || showWindow != m_IsVisualizingUI)
            {
                SetVisualizing(showWindow, showDrawing);
            }
        }

        public void SetVisualizing(bool ui, bool drawing)
        {
            m_IsVisualizingUI = ui;
            m_IsVisualizingDrawing = drawing;

            if (m_VisualWindow != null)
            {
                m_VisualWindow.SetActive(ui);
            }
            else if (ui)
            {
                m_VisualWindow = new HudWindow(Title);
                HudPanel.AddWindow(m_VisualWindow);
            }

            if ((ui || drawing) && m_Visual == null)
            {
                m_Visual = GetVisualFactory().GetOrCreateVisual(Topic);
            }

            if (m_Visual != null)
            {
                m_Visual.SetDrawingEnabled(drawing);

                if (m_VisualWindow != null)
                {
                    m_VisualWindow.SetOnGUI(m_Visual.OnGUI);
                }
            }
        }

        [Serializable]
        public class SaveState
        {
            public string Topic;
            public MessageSubtopic Subtopic;
            public string RosMessageName;
            public Rect Rect;
            public bool HasRect;
            public bool IsService;
            public bool ShowWindow;
            public bool ShowDrawing;
        }

        public VisualizationTopicsTabEntry(RosTopicState baseState, Texture2D background)
        {
            m_TopicState = baseState;
            m_Background = background;

            if (baseState.ServiceResponseTopic != null)
            {
                m_ServiceResponseTopic = new VisualizationTopicsTabEntry(baseState.ServiceResponseTopic, background);
            }

            m_CachedRosMessageName = RosMessageName;
        }

        public void LoadSaveState(SaveState save)
        {
            if (m_TopicState.ServiceResponseTopic != null && save.Subtopic == MessageSubtopic.Response)
            {
                // this save actually applies to the response topic
                m_ServiceResponseTopic.LoadSaveState(save);
                return;
            }

            if (save.HasRect && save.Rect.width > 0 && save.Rect.height > 0)
            {
                m_VisualWindow = new HudWindow(Title, save.Rect);
            }
            else if (save.ShowWindow)
            {
                m_VisualWindow = new HudWindow(Title);
            }

            if (m_VisualWindow != null)
            {
                HudPanel.AddWindow(m_VisualWindow);
            }

            SetVisualizing(save.ShowWindow, save.ShowDrawing);
        }

        void InitLineStyle()
        {
            m_LineStyle = new GUIStyle(GUI.skin.label);
            m_LineStyle.hover.textColor = Color.white;

            m_HoverStyle = new GUIStyle(GUI.skin.label);
            m_HoverStyle.hover.textColor = Color.white;
            m_HoverStyle.hover.background = m_Background;
            m_HoverStyle.normal.background = m_Background;
        }

        public void AddSaveStates(List<VisualizationTopicsTabEntry.SaveState> results)
        {
            if (IsVisualizingUI || IsVisualizingDrawing)
            {
                results.Add(new SaveState
                {
                    Topic = m_TopicState.Topic,
                    Subtopic = m_TopicState.Subtopic,
                    RosMessageName = m_TopicState.RosMessageName,
                    Rect = m_VisualWindow != null ? m_VisualWindow.WindowRect : new Rect(0, 0, 0, 0),
                    HasRect = m_VisualWindow != null,
                    IsService = m_TopicState.ServiceResponseTopic != null || m_TopicState.Subtopic == MessageSubtopic.Response,
                    ShowWindow = m_IsVisualizingUI,
                    ShowDrawing = m_IsVisualizingDrawing,
                });
            }

            if (m_ServiceResponseTopic != null)
            {
                m_ServiceResponseTopic.AddSaveStates(results);
            }
        }

#if UNITY_EDITOR
        void ShowOptionsMenu(Rect position)
        {
            UnityEditor.GenericMenu menu = new UnityEditor.GenericMenu();
            foreach(IVisualFactory factory in VisualFactoryRegistry.GetAllVisualFactories(Topic, RosMessageName))
            {
                bool isSelected = GetVisualFactory() == factory;
                menu.AddItem(new GUIContent(isSelected? factory.Name+" (main)": factory.Name), false, () => OnSelect(factory));
            }
            menu.DropDown(position);
        }

        static Action<IVisualFactory, string, string> s_OpenWindowCallback;

        public static void SetOpenWindowCallback(Action<IVisualFactory, string, string> callback)
        {
            s_OpenWindowCallback = callback;
        }

        void OnSelect(IVisualFactory factory)
        {
            if (s_OpenWindowCallback != null)
                s_OpenWindowCallback(factory, Topic, RosMessageName);
        }
#endif
    }
}
