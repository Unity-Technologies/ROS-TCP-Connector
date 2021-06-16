using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.MessageVisualizers;
using System.IO;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IHudTab
    {
        string Label { get; }
        void OnGUI(HUDPanel hud);
        void OnSelected();
        void OnDeselected();
    }

    public class HUDPanel : MonoBehaviour
    {
        public bool IsEnabled { get; set; }

        // GUI variables
        static bool s_Initialized = false; 
        public static GUIStyle s_IPStyle;
        public static GUIStyle s_BoldStyle;
        static Dictionary<string, string> s_MessageNamesByTopic = new Dictionary<string, string>();
        static SortedList<string, TopicVisualizationState> s_AllTopics = new SortedList<string, TopicVisualizationState>();
        
        public static string GetMessageNameByTopic(string topic)
        {
            if (!s_MessageNamesByTopic.TryGetValue(topic, out var rosMessageName))
            {
                return null;
            }

            return rosMessageName;
        }

        // ROS Message variables
        string m_RosConnectAddress = "";

        int m_NextServiceID = 101;
        int m_NextWindowID = 101;
        int m_CurrentFrameIndex;

        List<TopicVisualizationState> m_ActiveWindows = new List<TopicVisualizationState>();
        Dictionary<string, IVisualFactory> m_TopicVisualizers = new Dictionary<string, IVisualFactory>();
        Dictionary<int, TopicVisualizationState> m_PendingServiceRequests = new Dictionary<int, TopicVisualizationState>();
        TopicVisualizationState m_DraggingWindow;

        public static SortedList<string, TopicVisualizationState> AllTopics => s_AllTopics;
        static List<IHudTab> s_HUDTabs = new List<IHudTab> { new TopicsHudTab() };

        public static void RegisterTab(IHudTab tab)
        {
            s_HUDTabs.Add(tab);
        }

        IHudTab m_SelectedTab;

        float m_MessageOutLastRealtime;
        float m_MessageInLastRealtime;

        string LayoutFilePath => Path.Combine(Application.persistentDataPath, "RosHudLayout.json");

        void SaveLayout()
        {
            HUDLayoutSave saveState = new HUDLayoutSave { };
            saveState.AddRules(s_AllTopics.Values);
            File.WriteAllText(LayoutFilePath, JsonUtility.ToJson(saveState));
        }

        void LoadLayout()
        {
            if (File.Exists(LayoutFilePath))
                LoadLayout(JsonUtility.FromJson<HUDLayoutSave>(File.ReadAllText(LayoutFilePath)));
        }

        void LoadLayout(HUDLayoutSave saveState)
        {
            m_ActiveWindows.Clear();
            RequestTopics();
            foreach (TopicVisualizationState.SaveState savedRule in saveState.Rules)
            {
                s_AllTopics[savedRule.Topic] = new TopicVisualizationState(savedRule, this);
            }
        }

        public void AddWindow(TopicVisualizationState window)
        {
            m_ActiveWindows.Add(window);
        }

        public void RemoveWindow(TopicVisualizationState window)
        {
            m_ActiveWindows.Remove(window);
        }

        public int GetNextWindowID()
        {
            int result = m_NextWindowID;
            m_NextWindowID++;
            return result;
        }

        public void SetLastMessageSent(string topic, Message message)
        {
            if (topic.StartsWith("__"))
                return;

            TopicVisualizationState state;
            if (!s_AllTopics.TryGetValue(topic, out state))
            {
                s_AllTopics.Add(topic, null);
            }
            if (state != null)
                state.SetMessage(message, new MessageMetadata(topic, m_CurrentFrameIndex, DateTime.Now));
            m_MessageOutLastRealtime = Time.realtimeSinceStartup;
        }

        public void SetLastMessageReceived(string topic, Message message)
        {
            if (topic.StartsWith("__"))
                return;

            TopicVisualizationState state;
            if (!s_AllTopics.TryGetValue(topic, out state))
            {
                s_AllTopics.Add(topic, null);
            }
            if (state != null)
                state.SetMessage(message, new MessageMetadata(topic, m_CurrentFrameIndex, DateTime.Now));
            m_MessageInLastRealtime = Time.realtimeSinceStartup;
        }

        public int AddServiceRequest(string topic, Message request)
        {
            int serviceID = m_NextServiceID;
            m_NextServiceID++;

            MessageMetadata meta = new MessageMetadata(topic, m_CurrentFrameIndex, DateTime.Now);

            TopicVisualizationState state;
            if (!s_AllTopics.TryGetValue(topic, out state))
            {
                s_AllTopics.Add(topic, null);
            }
            if (state != null)
            {
                m_PendingServiceRequests.Add(serviceID, state);
                state.SetServiceRequest(request, new MessageMetadata(topic, m_CurrentFrameIndex, DateTime.Now), serviceID);
            }
            return serviceID;
        }

        public void AddServiceResponse(int serviceID, Message response)
        {
            TopicVisualizationState state;
            if (!m_PendingServiceRequests.TryGetValue(serviceID, out state))
                return; // don't know what happened there, but that's not a request I recognize

            m_PendingServiceRequests.Remove(serviceID);

            if (state != null)
            {
                state.SetServiceResponse(response, new MessageMetadata(state.Topic, m_CurrentFrameIndex, DateTime.Now), serviceID);
            }
        }

        void InitStyles()
        {
            if (s_Initialized)
                return;

            s_Initialized = true;
            s_IPStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
            };

            s_BoldStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
                fontStyle = FontStyle.Bold,
            };
        }

        void Awake()
        {
            InitStyles();

            LoadLayout();
        }

        void OnApplicationQuit()
        {
            SaveLayout();
        }

        public void SetRosIP(string ip, int port)
        {
            m_RosConnectAddress = $"{ip}:{port}";
        }

        public void LateUpdate()
        {
            // used to track whether a given visualization has already updated this frame
            m_CurrentFrameIndex = (m_CurrentFrameIndex + 1)%1000000;
        }

        Color GetConnectionColor(float elapsedTime)
        {
            var bright = new Color(1, 1, 0.5f);
            var mid = new Color(0, 1, 1);
            var dark = new Color(0, 0.5f, 1);
            const float brightDuration = 0.03f;
            const float fadeToDarkDuration = 1.0f;

            if (!ROSConnection.instance.HasConnectionThread)
                return Color.gray;
            if (ROSConnection.instance.HasConnectionError)
                return Color.red;

            if (elapsedTime <= brightDuration)
                return bright;
            else
                return Color.Lerp(mid, dark, elapsedTime/fadeToDarkDuration);
        }

        void OnGUI()
        {
            if (!IsEnabled)
                return;

            // Initialize main HUD
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(300));

            // ROS IP Setup
            GUILayout.BeginHorizontal();
            Color baseColor = GUI.color;
            GUI.color = GetConnectionColor(Time.realtimeSinceStartup - m_MessageInLastRealtime);
            GUILayout.Label("\u25C0", s_BoldStyle, GUILayout.Width(10));
            GUI.color = GetConnectionColor(Time.realtimeSinceStartup - m_MessageOutLastRealtime);
            GUILayout.Label("\u25B6", s_BoldStyle, GUILayout.Width(15));
            GUI.color = baseColor;
            GUILayout.Label("ROS IP: ", s_BoldStyle, GUILayout.Width(100));

            if (!ROSConnection.instance.HasConnectionThread)
            {
                ROSConnection.instance.RosIPAddress = GUILayout.TextField(ROSConnection.instance.RosIPAddress);
                GUILayout.EndHorizontal();
                GUILayout.Label("(Not connected)");
                if (GUILayout.Button("Connect"))
                    ROSConnection.instance.Connect();
            }
            else
            {
                GUILayout.Label(m_RosConnectAddress, s_IPStyle);
                GUILayout.EndHorizontal();
            }

            GUILayout.BeginHorizontal();
            foreach(IHudTab tab in s_HUDTabs)
            {
                bool wasSelected = tab == m_SelectedTab;
                bool selected = GUILayout.Toggle(wasSelected, tab.Label, GUI.skin.button);
                if(selected != wasSelected)
                {
                    if(m_SelectedTab != null)
                        m_SelectedTab.OnDeselected();

                    m_SelectedTab = selected? tab: null;

                    if(m_SelectedTab != null)
                        m_SelectedTab.OnSelected();
                }
            }
            GUILayout.EndHorizontal();

            if (m_SelectedTab != null)
                m_SelectedTab.OnGUI(this);

            GUILayout.EndVertical();

            // Draggable windows
            Event current = Event.current;
            if (current.type == EventType.MouseDown)
            {
                for (int Idx = m_ActiveWindows.Count - 1; Idx >= 0; --Idx)
                {
                    TopicVisualizationState window = m_ActiveWindows[Idx];
                    if (m_ActiveWindows[Idx].TryDragWindow(current))
                    {
                        m_DraggingWindow = m_ActiveWindows[Idx];
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

            foreach (TopicVisualizationState window in m_ActiveWindows)
            {
                window.DrawWindow();
            }
        }

        public static Rect GetDefaultWindowRect()
        {
            return new Rect(300, 0, 300, 200);
        }

        public Rect GetFreeWindowRect()
        {
            Queue<Rect> xQueue = new Queue<Rect>();
            Queue<Rect> yQueue = new Queue<Rect>();
            yQueue.Enqueue(GetDefaultWindowRect());

            while(yQueue.Count > 0 || xQueue.Count > 0)
            {
                Rect testRect = xQueue.Count > 0? xQueue.Dequeue(): yQueue.Dequeue();
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
            foreach(TopicVisualizationState window in m_ActiveWindows)
            {
                if (window.WindowRect.Overlaps(rect))
                    return false;
            }
            return true;
        }

        public bool IsFreeWindowRect(Rect rect, out float maxX, out float maxY)
        {
            maxX = 0;
            maxY = 0;
            bool result = true;
            foreach (TopicVisualizationState window in m_ActiveWindows)
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

        public IVisualFactory GetVisualizer(string topic)
        {
            IVisualFactory result;
            if (m_TopicVisualizers.TryGetValue(topic, out result))
            {
                return result;
            }

            string rosMessageName = GetMessageNameByTopic(topic);
            result = VisualFactoryRegistry.GetVisualizer(topic, rosMessageName);
            m_TopicVisualizers[topic] = result;
            return result;
        }

        public TopicVisualizationState GetVisualizationState(string topic, bool subscribe=false)
        {
            if (s_AllTopics.TryGetValue(topic, out var result) && result != null)
            {
                return result;
            }

            string rosMessageName = GetMessageNameByTopic(topic);
            if (rosMessageName != null)
            {
                result = new TopicVisualizationState(topic, rosMessageName, this, subscribe);
                s_AllTopics[topic] = result;    
            }
            return result;
        }

        float m_LastTopicsRequestRealtime = -1;
        const float k_TimeBetweenTopicsUpdates = 5.0f;

        public void RequestTopics()
        {
            if (m_LastTopicsRequestRealtime == -1 || (Time.realtimeSinceStartup - m_LastTopicsRequestRealtime) > k_TimeBetweenTopicsUpdates)
            {
                ROSConnection.instance.GetTopicList(RegisterTopics);
                m_LastTopicsRequestRealtime = Time.realtimeSinceStartup;
            }
        }

        void RegisterTopics(string[] topics, string[] messageNames)
        {
            for (int Idx = 0; Idx < topics.Length; ++Idx)
            {
                string topic = topics[Idx];
                if (!s_AllTopics.ContainsKey(topic))
                {
                    s_AllTopics.Add(topic, null);
                }
                s_MessageNamesByTopic[topic] = messageNames[Idx];
            }
            //ResizeTopicsWindow();
            m_TopicVisualizers.Clear(); // update to the newest message types
        }


        class HUDLayoutSave
        {
            public TopicVisualizationState.SaveState[] Rules;

            public void AddRules(IEnumerable<TopicVisualizationState> rules)
            {
                List<TopicVisualizationState.SaveState> topicRuleSaves = new List<TopicVisualizationState.SaveState>();
                foreach (TopicVisualizationState rule in rules)
                {
                    if (rule == null)
                        continue;
                    TopicVisualizationState.SaveState save = rule.CreateSaveState();
                    if (save != null)
                        topicRuleSaves.Add(save);
                }
                this.Rules = topicRuleSaves.ToArray();
            }
        }

        class TopicsHudTab : IHudTab
        {
            string IHudTab.Label => "Topics";

            Vector2 m_TopicMenuScrollPosition;
            string m_TopicFilter = "";

            public void OnSelected() { }
            public void OnDeselected() { }

            public void OnGUI(HUDPanel hud)
            {
                hud.RequestTopics();

                GUILayout.BeginHorizontal();
                m_TopicFilter = GUILayout.TextField(m_TopicFilter);

                if (m_TopicFilter != "" && !AllTopics.ContainsKey(m_TopicFilter))
                {
                    if (GUILayout.Button($"Subscribe to \"{m_TopicFilter}\""))
                    {
                        TopicVisualizationState state = new TopicVisualizationState(m_TopicFilter, GetMessageNameByTopic(m_TopicFilter), hud);
                        state.SetShowWindow(true);
                        state.SetShowDrawing(true);
                        AllTopics.Add(m_TopicFilter, state);
                    }
                }
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("UI", HUDPanel.s_BoldStyle, GUILayout.Width(20));
                GUILayout.Label("Viz", HUDPanel.s_BoldStyle);
                GUILayout.EndHorizontal();

                m_TopicMenuScrollPosition = GUILayout.BeginScrollView(m_TopicMenuScrollPosition);
                int numTopicsShown = 0;
                foreach (KeyValuePair<string, TopicVisualizationState> kv in AllTopics)
                {
                    bool showWindow = false;
                    bool canShowWindow = false;
                    bool showDrawing = false;
                    bool canShowDrawing = false;
                    string title = kv.Key;
                    if (!title.Contains(m_TopicFilter))
                    {
                        continue;
                    }
                    string rosMessageName = GetMessageNameByTopic(title);

                    numTopicsShown++;
                    TopicVisualizationState state = kv.Value;

                    if (state != null)
                    {
                        showWindow = state.ShowWindow;
                        showDrawing = state.ShowDrawing;
                        title = state.Topic;
                    }

                    IVisualFactory visualFactory = hud.GetVisualizer(kv.Key);
                    canShowWindow = visualFactory != null;
                    canShowDrawing = visualFactory != null ? visualFactory.CanShowDrawing : false;

                    bool hasWindow = showWindow;
                    bool hasDrawing = showDrawing;

                    GUILayout.BeginHorizontal();
                    if (hasWindow || canShowWindow)
                        showWindow = GUILayout.Toggle(showWindow, "", GUILayout.Width(15));
                    else
                        GUILayout.Label("", GUILayout.Width(15));

                    if (hasDrawing || canShowDrawing)
                        showDrawing = GUILayout.Toggle(showDrawing, "", GUILayout.Width(15));
                    else
                        GUILayout.Label("", GUILayout.Width(15));

                    Color baseColor = GUI.color;
                    GUI.color = canShowWindow ? baseColor : Color.grey;
                    if (GUILayout.Button(new GUIContent(title, rosMessageName), GUI.skin.label, GUILayout.Width(240)))
                    {
                        if (!canShowWindow)
                        {
                            Debug.LogError($"No message class registered for type {rosMessageName}");
                        }
                        else if (!canShowDrawing)
                        {
                            showWindow = !showWindow;
                        }
                        else
                        {
                            bool toggleOn = (!showWindow || !showDrawing);
                            showWindow = toggleOn;
                            showDrawing = toggleOn;
                        }
                    }
                    GUI.color = baseColor;
                    GUILayout.EndHorizontal();

                    if (showWindow != hasWindow || showDrawing != hasDrawing)
                    {
                        if (state == null)
                        {
                            state = new TopicVisualizationState(kv.Key, GetMessageNameByTopic(kv.Key), hud);
                            AllTopics[kv.Key] = state;
                        }
                        state.SetShowWindow(showWindow);
                        state.SetShowDrawing(showDrawing);
                        break;
                    }
                }
                GUILayout.EndScrollView();

                if (numTopicsShown == 0)
                {
                    if (AllTopics.Count == 0)
                        GUILayout.Label("No topics registered");
                    else
                        GUILayout.Label($"No topics named \"{m_TopicFilter}\"!");
                }
            }
        }
    }
}
