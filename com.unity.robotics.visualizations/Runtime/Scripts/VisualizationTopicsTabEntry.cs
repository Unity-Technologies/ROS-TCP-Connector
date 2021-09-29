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

        List<VisualRow> m_VisualRows = new List<VisualRow>();

        public class VisualRow
        {
            VisualizationTopicsTabEntry m_Entry;

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

            [Serializable]
            public class SaveState
            {
                public string VisualizerID;
                public Rect Rect;
                public bool HasRect;
                public bool ShowWindow;
                public bool ShowDrawing;
            }

            public SaveState CreateSaveState()
            {
                return new SaveState()
                {
                    Rect = m_VisualWindow != null ? m_VisualWindow.WindowRect : new Rect(0, 0, 0, 0),
                    HasRect = m_VisualWindow != null,
                    VisualizerID = m_VisualFactory.ID,
                    ShowWindow = m_IsVisualizingUI,
                    ShowDrawing = m_IsVisualizingDrawing,
                };
            }

            public VisualRow(VisualizationTopicsTabEntry entry, IVisualFactory visualizer)
            {
                m_Entry = entry;
                m_VisualFactory = visualizer;
                m_CachedRosMessageName = m_Entry.RosMessageName;
            }

            public VisualRow(VisualizationTopicsTabEntry entry, SaveState save = null)
            {
                m_Entry = entry;
                if (save != null)
                {
                    if (save.VisualizerID != null)
                    {
                        m_VisualFactory = VisualFactoryRegistry.GetAllVisualFactories(m_Entry.Topic, m_Entry.RosMessageName).FirstOrDefault(v => v.ID == save.VisualizerID);
                        m_CachedRosMessageName = m_Entry.RosMessageName;
                    }

                    if (save.HasRect && save.Rect.width > 0 && save.Rect.height > 0)
                    {
                        m_VisualWindow = new HudWindow(entry.Title, save.Rect);
                    }
                    else if (save.ShowWindow)
                    {
                        m_VisualWindow = new HudWindow(entry.Title);
                    }

                    if (m_VisualWindow != null)
                    {
                        HudPanel.AddWindow(m_VisualWindow);
                    }

                    SetVisualizing(save.ShowWindow, save.ShowDrawing);
                }
            }

            public IVisualFactory GetVisualFactory()
            {
                if (m_CachedRosMessageName != m_Entry.RosMessageName)
                {
                    // if the topic has changed, discard our cached data
                    m_VisualFactory = null;
                    m_NoVisualFactoryAvailable = false;
                }

                if (m_VisualFactory == null && !m_NoVisualFactoryAvailable)
                {
                    SetVisualFactory(VisualFactoryRegistry.GetVisualFactory(m_Entry.Topic, m_Entry.RosMessageName, m_Entry.Subtopic));
                }
                return m_VisualFactory;
            }

            public void SetVisualFactory(IVisualFactory visualFactory)
            {
                if (m_Visual != null)
                    m_Visual.SetDrawingEnabled(false);
                m_Visual = null;

                m_VisualFactory = visualFactory;
                m_CachedRosMessageName = m_Entry.RosMessageName;
                if (m_VisualFactory == null)
                    m_NoVisualFactoryAvailable = true;

                SetVisualizing(m_IsVisualizingUI, m_IsVisualizingDrawing);
            }

            public bool CanShowWindow => GetVisualFactory() != null;
            public bool CanShowDrawing => GetVisualFactory() != null && m_VisualFactory.CanShowDrawing;

            public void DrawGUILine()
            {
                bool showWindow = IsVisualizingUI;
                bool showDrawing = IsVisualizingDrawing;

                bool canShowWindow = CanShowWindow;
                bool canShowDrawing = CanShowDrawing;

                var hasWindow = showWindow;
                var hasDrawing = showDrawing;

                if (m_Entry.m_LineStyle == null)
                    m_Entry.InitLineStyle();

                GUILayout.BeginHorizontal(m_IsHovering ? m_Entry.m_HoverStyle : m_Entry.m_LineStyle);
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
                    Time.realtimeSinceStartup - m_Entry.TopicState.LastMessageReceivedRealtime,
                    Time.realtimeSinceStartup - m_Entry.TopicState.LastMessageSentRealtime,
                    m_Entry.TopicState.IsPublisher,
                    m_Entry.TopicState.SentSubscriberRegistration,
                    m_Entry.m_TopicState.Connection.HasConnectionError);

                if (GUILayout.Button(m_Entry.Title, m_Entry.m_LineStyle, GUILayout.Width(240)))
                {
                    if (!canShowWindow)
                    {
                        Debug.LogError($"No message class registered for type {m_Entry.RosMessageName}");
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
                    m_Entry.ShowOptionsMenu(buttonRect);
#endif

                if (m_IsHovering)
                {
                    string labelText = m_Entry.RosMessageName;
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
                if (m_VisualWindow != null)
                {
                    m_VisualWindow.SetActive(ui);
                }
                else if (ui)
                {
                    m_VisualWindow = new HudWindow(m_Entry.Topic);
                    HudPanel.AddWindow(m_VisualWindow);
                }

                if ((ui || drawing) && m_Visual == null)
                {
                    m_Visual = GetVisualFactory().GetOrCreateVisual(m_Entry.Topic);
                }

                if (m_Visual != null)
                {
                    m_Visual.SetDrawingEnabled(drawing);

                    if (m_VisualWindow != null)
                    {
                        m_VisualWindow.SetOnGUI(m_Visual.OnGUI);
                    }
                }

                m_IsVisualizingUI = ui;
                m_IsVisualizingDrawing = drawing;
            }
        }

        [Serializable]
        public class SaveState
        {
            public string Topic;
            public string RosMessageName;
            public List<VisualRow.SaveState> Rows;
        }

        public VisualizationTopicsTabEntry(RosTopicState baseState, Texture2D background)
        {
            m_TopicState = baseState;
            m_Background = background;
            m_VisualRows.Add(new VisualRow(this));

            if (baseState.ServiceResponseTopic != null)
            {
                m_ServiceResponseTopic = new VisualizationTopicsTabEntry(baseState.ServiceResponseTopic, background);
            }
        }

        internal VisualizationTopicsTabEntry(SaveState save, RosTopicState topicState, Texture2D background)
        {
            m_TopicState = topicState;
            m_Background = background;
            foreach (VisualRow.SaveState rowSave in save.Rows)
            {
                m_VisualRows.Add(new VisualRow(this, save: rowSave));
            }
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

        public SaveState CreateSaveState()
        {
            if (!m_VisualRows.Any(v => v.IsVisualizingUI || v.IsVisualizingDrawing))
            {
                return null;
            }

            return new SaveState
            {
                Topic = m_TopicState.Topic,
                RosMessageName = m_TopicState.RosMessageName,
                Rows = m_VisualRows.Select(v => v.CreateSaveState()).ToList()
            };
        }

        public void DrawGUI()
        {
            foreach (VisualRow row in m_VisualRows)
            {
                row.DrawGUILine();
            }

            if (m_ServiceResponseTopic != null)
            {
                m_ServiceResponseTopic.DrawGUI();
            }
        }

#if UNITY_EDITOR
        void ShowOptionsMenu(Rect position)
        {
            UnityEditor.GenericMenu menu = new UnityEditor.GenericMenu();
            foreach(IVisualFactory factory in VisualFactoryRegistry.GetAllVisualFactories(Topic, RosMessageName))
            {
                bool isSelected = m_VisualRows.Any(r => r.GetVisualFactory() == factory);
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
