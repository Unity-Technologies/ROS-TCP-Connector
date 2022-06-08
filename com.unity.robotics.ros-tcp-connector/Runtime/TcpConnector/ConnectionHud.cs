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
        void OnGUI(ConnectionHud hud);
        void OnSelected();
        void OnDeselected();
    }

    public class ConnectionHud : MonoBehaviour
    {
        public static GUIStyle s_BoldStyle;

        // these are static so that anyone can register tabs and windows without needing to worry about whether the hud has been initialized
        static SortedList<int, IHudTab> s_HudTabs = new SortedList<int, IHudTab>();
        static SortedList<int, Action> s_HeaderContents = new SortedList<int, Action>();
        static List<HudWindow> s_ActiveWindows = new List<HudWindow>();
        static int s_NextWindowID = 101;
        static ConnectionHud s_Instance;

        internal bool isEnabled = true;

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

            foreach (IHudTab tab in s_HudTabs.Values)
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
            if (!Application.isPlaying)
                return;

            if (s_HudTabs.ContainsKey(index))
            {
                Debug.LogWarning($"HUDPanel already contains a tab registered at index {index}! Registering at index {s_HudTabs.Count} instead.");
                index = s_HudTabs.Count;
            }

            s_HudTabs.Add(index, tab);
            Instantiate();
        }

        public static void RegisterHeader(Action headerContent, int index = 0)
        {
            if (!Application.isPlaying)
                return;

            if (s_HeaderContents.ContainsKey(index))
            {
                Debug.LogWarning($"HUDPanel already contains a header registered at index {index}! Registering at index {s_HeaderContents.Count} instead.");
                index = s_HeaderContents.Count;
            }

            s_HeaderContents.Add(index, headerContent);
            Instantiate();
        }

        public static void Instantiate()
        {
            if (!Application.isPlaying)
                return;

            if (s_Instance == null)
            {
                GameObject gameObject = new GameObject("ConnectionHud");
                s_Instance = gameObject.AddComponent<ConnectionHud>();
            }
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
                s_HudTabs.Clear();
                s_HeaderContents.Clear();
                s_ActiveWindows.Clear();
                s_Instance = null;
            }
        }
#endif

        static GUIStyle s_ConnectionArrowStyle;

        public static void DrawConnectionArrows(bool withBar, float x, float y, float receivedTime, float sentTime, bool isPublisher, bool isSubscriber, bool hasError)
        {
            if (s_ConnectionArrowStyle == null)
            {
                s_ConnectionArrowStyle = new GUIStyle
                {
                    alignment = TextAnchor.MiddleLeft,
                    normal = { textColor = Color.white },
                    fontSize = 22,
                    fontStyle = FontStyle.Bold,
                    fixedWidth = 250
                };
            }

            var baseColor = GUI.color;
            GUI.color = Color.white;
            if (withBar)
                GUI.Label(new Rect(x + 4, y + 5, 25, 15), "I", s_ConnectionArrowStyle);
            GUI.color = GetConnectionColor(receivedTime, isSubscriber, hasError);
            GUI.Label(new Rect(x + 8, y + 6, 25, 15), "\u2190", s_ConnectionArrowStyle);
            GUI.color = GetConnectionColor(sentTime, isPublisher, hasError);
            GUI.Label(new Rect(x + 8, y + 0, 25, 15), "\u2192", s_ConnectionArrowStyle);
            GUI.color = baseColor;
        }

        public static Color GetConnectionColor(float elapsedTime, bool hasConnection, bool hasError)
        {
            var bright = new Color(1, 1, 0.5f);
            var mid = new Color(0, 1, 1);
            var dark = new Color(0, 0.5f, 1);
            const float brightDuration = 0.03f;
            const float fadeToDarkDuration = 1.0f;

            if (!hasConnection)
                return Color.gray;
            if (hasError)
                return Color.red;

            if (elapsedTime <= brightDuration)
                return bright;
            return Color.Lerp(mid, dark, elapsedTime / fadeToDarkDuration);
        }
    }
}
