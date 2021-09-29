using System;
using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IHudTab
    {
        string Label { get; }
        void OnGUI(HudPanel hud);
        void OnSelected();
        void OnDeselected();
    }

    public class HudPanel : MonoBehaviour
    {
        public static GUIStyle s_BoldStyle;

        // these are static so that anyone can register tabs and windows without needing to worry about whether the hud has been initialized
        static SortedList<int, IHudTab> s_HUDTabs = new SortedList<int, IHudTab>();
        static SortedList<int, Action> s_HeaderContents = new SortedList<int, Action>();
        static List<HudWindow> s_ActiveWindows = new List<HudWindow>();
        static int s_NextWindowID = 101;

        // ROS Message variables
        internal bool isEnabled;

        HudWindow m_DraggingWindow;

        IHudTab m_SelectedTab;

        void Awake()
        {
            InitStyles();
        }

        void OnGUI()
        {
            if (!isEnabled)
                return;

            // Initialize main HUD
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(300));

            foreach (Action element in s_HeaderContents.Values)
            {
                element();
            }

            GUILayout.BeginHorizontal();

            foreach (IHudTab tab in s_HUDTabs.Values)
            {
                var wasSelected = tab == m_SelectedTab;
                var selected = GUILayout.Toggle(wasSelected, tab.Label, GUI.skin.button);
                if (selected != wasSelected)
                {
                    if (m_SelectedTab != null)
                        m_SelectedTab.OnDeselected();

                    m_SelectedTab = selected ? tab : null;

                    if (m_SelectedTab != null)
                        m_SelectedTab.OnSelected();
                }
            }

            GUILayout.EndHorizontal();

            if (m_SelectedTab != null)
                m_SelectedTab.OnGUI(this);

            GUILayout.EndVertical();

            // Draggable windows
            var current = Event.current;
            if (current.type == EventType.MouseDown)
            {
                for (var Idx = s_ActiveWindows.Count - 1; Idx >= 0; --Idx)
                {
                    var window = s_ActiveWindows[Idx];
                    if (s_ActiveWindows[Idx].TryDragWindow(current))
                    {
                        m_DraggingWindow = s_ActiveWindows[Idx];
                        break;
                    }
                }
            }
            else if (current.type == EventType.MouseDrag && m_DraggingWindow != null)
            {
                m_DraggingWindow.UpdateDragWindow(current);
            }
            else if (current.type == EventType.MouseUp && m_DraggingWindow != null)
            {
                m_DraggingWindow.EndDragWindow();
                m_DraggingWindow = null;
            }

            foreach (var window in s_ActiveWindows)
                window.DrawWindow(this);
        }

        public static void RegisterTab(IHudTab tab, int index = 0)
        {
            if (s_HUDTabs.ContainsKey(index))
            {
                Debug.LogWarning($"HUDPanel already contains a tab registered at index {index}! Registering at index {s_HUDTabs.Count} instead.");
                index = s_HUDTabs.Count;
            }

            s_HUDTabs.Add(index, tab);
        }

        public static void RegisterHeader(Action headerContent, int index = 0)
        {
            if (s_HeaderContents.ContainsKey(index))
            {
                Debug.LogWarning($"HUDPanel already contains a header registered at index {index}! Registering at index {s_HeaderContents.Count} instead.");
                index = s_HeaderContents.Count;
            }

            s_HeaderContents.Add(index, headerContent);
        }

        public static void AddWindow(HudWindow window)
        {
            s_ActiveWindows.Add(window);
        }

        public static void RemoveWindow(HudWindow window)
        {
            s_ActiveWindows.Remove(window);
        }

        public static int GetNextWindowID()
        {
            var result = s_NextWindowID;
            s_NextWindowID++;
            return result;
        }

        void InitStyles()
        {
            // Define font styles
            s_BoldStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
                fontStyle = FontStyle.Bold,
            };
        }
        public static Rect GetDefaultWindowRect()
        {
            return new Rect(450, 0, 300, 200);
        }

        public Rect GetFreeWindowRect()
        {
            var xQueue = new Queue<Rect>();
            var yQueue = new Queue<Rect>();
            yQueue.Enqueue(GetDefaultWindowRect());

            while (yQueue.Count > 0 || xQueue.Count > 0)
            {
                var testRect = xQueue.Count > 0 ? xQueue.Dequeue() : yQueue.Dequeue();
                if (testRect.xMax > Screen.width || testRect.yMax > Screen.height)
                    continue;

                float maxX, maxY;
                if (IsFreeWindowRect(testRect, out maxX, out maxY))
                    return testRect;

                xQueue.Enqueue(new Rect(maxX, testRect.y, testRect.width, testRect.height));
                yQueue.Enqueue(new Rect(testRect.x, maxY, testRect.width, testRect.height));
            }

            return GetDefaultWindowRect();
        }

        public bool IsFreeWindowRect(Rect rect)
        {
            foreach (var window in s_ActiveWindows)
                if (window.WindowRect.Overlaps(rect))
                    return false;
            return true;
        }

        public bool IsFreeWindowRect(Rect rect, out float maxX, out float maxY)
        {
            maxX = 0;
            maxY = 0;
            var result = true;
            foreach (var window in s_ActiveWindows)
            {
                if (window.WindowRect.Overlaps(rect))
                {
                    maxX = Mathf.Max(maxX, window.WindowRect.xMax);
                    maxY = Mathf.Max(maxY, window.WindowRect.yMax);
                    result = false;
                }
            }

            return result;
        }

#if UNITY_EDITOR
        // automatically clear all static content on pressing play
        [UnityEditor.InitializeOnLoadMethod]
        static void InitializeOnLoad()
        {
            UnityEditor.EditorApplication.playModeStateChanged += OnPlayMode;
        }

        static void OnPlayMode(UnityEditor.PlayModeStateChange change)
        {
            if (change == UnityEditor.PlayModeStateChange.ExitingEditMode)
            {
                s_HUDTabs.Clear();
                s_HeaderContents.Clear();
                s_ActiveWindows.Clear();
            }
        }
#endif
    }
}
