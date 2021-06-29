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
        // static bool s_Initialized = false;
        // public static GUIStyle s_IPStyle;
        // public static GUIStyle s_BoldStyle;
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
        GUIStyle m_LabelStyle;
        static GUIStyle m_ConnectionArrowStyle;
        GUIStyle m_ContentStyle;
        GUIStyle m_MessageStyle;
        bool m_ViewSent = false;
        bool m_ViewRecv = false;
        bool m_ViewSrvs = false;
        Rect m_ScrollRect;

        // ROS Message variables
        internal bool isEnabled;
        internal string host;

        MessageViewState m_LastMessageSent;
        string m_LastMessageSentMeta = "None";
        float m_LastMessageSentRealtime;

        // // ROS Message variables
        // string m_RosConnectAddress = "";

        List<TopicVisualizationState> m_ActiveWindows = new List<TopicVisualizationState>();
        Dictionary<string, IVisualFactory> m_TopicVisualizers = new Dictionary<string, IVisualFactory>();
        Dictionary<int, TopicVisualizationState> m_PendingServiceRequests = new Dictionary<int, TopicVisualizationState>();
        TopicVisualizationState m_DraggingWindow;

        public static SortedList<string, TopicVisualizationState> AllTopics => s_AllTopics;
        static List<IHudTab> s_HUDTabs = new List<IHudTab> { new TopicsHudTab() };

        // For the Hud's IP address field, we store the IP address and port in PlayerPrefs.
        // This is used to remember the last IP address the player typed into the HUD, in builds where ConnectOnStart is not checked
        public const string PlayerPrefsKey_ROS_IP = "ROS_IP";
        public const string PlayerPrefsKey_ROS_TCP_PORT = "ROS_TCP_PORT";

        public static string RosIPAddressPref
        {
            get => PlayerPrefs.GetString(PlayerPrefsKey_ROS_IP, "127.0.0.1");
        }

        public static int RosPortPref
        {
            get => PlayerPrefs.GetInt(PlayerPrefsKey_ROS_TCP_PORT, 10000);
        }

        public static void SetIPPref(string ipAddress)
        {
            PlayerPrefs.SetString(PlayerPrefsKey_ROS_IP, ipAddress);
        }

        public static void SetPortPref(int port)
        {
            PlayerPrefs.SetInt(PlayerPrefsKey_ROS_TCP_PORT, port);
        }

        public void SetLastMessageSent(string topic, Message message)
        {
            TopicVisualizationState state;
            if (!s_AllTopics.TryGetValue(topic, out state))
            {
                s_AllTopics.Add(topic, null);
            }

            m_LastMessageSent = new MessageViewState() { label = "Last Message Sent:", message = message };
            m_LastMessageSentMeta = $"{topic} (time: {System.DateTime.Now.TimeOfDay})";
            m_LastMessageSentRealtime = Time.realtimeSinceStartup;
        }

        MessageViewState m_LastMessageReceived;
        string m_LastMessageReceivedMeta = "None";
        float m_LastMessageReceivedRealtime;

        public void SetLastMessageReceived(string topic, Message message)
        {
            TopicVisualizationState state;
            if (!s_AllTopics.TryGetValue(topic, out state))
            {
                s_AllTopics.Add(topic, null);
            }

            m_LastMessageReceived = new MessageViewState() { label = "Last Message Received:", message = message };
            m_LastMessageReceivedMeta = $"{topic} (time: {System.DateTime.Now.TimeOfDay})";
            m_LastMessageReceivedRealtime = Time.realtimeSinceStartup;
        }

        List<MessageViewState> activeServices = new List<MessageViewState>();
        MessageViewState lastCompletedServiceRequest = null;
        MessageViewState lastCompletedServiceResponse = null;
        int nextServiceID = 101;
        int m_NextWindowID = 101;
        int m_CurrentFrameIndex;

        public static void RegisterTab(IHudTab tab)
        {
            s_HUDTabs.Add(tab);
        }

        IHudTab m_SelectedTab;

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

        // public void SetLastMessageSent(string topic, Message message)
        // {
        //     if (topic.StartsWith("__"))
        //         return;
        //
        //     TopicVisualizationState state;
        //     if (!s_AllTopics.TryGetValue(topic, out state))
        //     {
        //         s_AllTopics.Add(topic, null);
        //     }
        //     if (state != null)
        //         state.SetMessage(message, new MessageMetadata(topic, m_CurrentFrameIndex, DateTime.Now));
        //     m_MessageOutLastRealtime = Time.realtimeSinceStartup;
        // }

        // public void SetLastMessageReceived(string topic, Message message)
        // {
        //     if (topic.StartsWith("__"))
        //         return;
        //
        //     TopicVisualizationState state;
        //     if (!s_AllTopics.TryGetValue(topic, out state))
        //     {
        //         s_AllTopics.Add(topic, null);
        //     }
        //     if (state != null)
        //         state.SetMessage(message, new MessageMetadata(topic, m_CurrentFrameIndex, DateTime.Now));
        //     m_MessageInLastRealtime = Time.realtimeSinceStartup;
        // }

        public int AddServiceRequest(string topic, Message request)
        {
            int serviceID = nextServiceID;
            nextServiceID++;

            TopicVisualizationState state;
            if (!s_AllTopics.TryGetValue(topic, out state))
            {
                s_AllTopics.Add(topic, null);
            }
            activeServices.Add(new MessageViewState()
            {
                serviceID = serviceID,
                timestamp = Time.time,
                topic = topic,
                message = request,
                label = $"{topic} Service Requested",
            });

            return serviceID;
        }

        public void AddServiceResponse(int serviceID, Message response)
        {
            TopicVisualizationState state;
            if (!m_PendingServiceRequests.TryGetValue(serviceID, out state))
                return; // don't know what happened there, but that's not a request I recognize

            lastCompletedServiceRequest = activeServices.Find(s => s.serviceID == serviceID);
            activeServices.Remove(lastCompletedServiceRequest);

            lastCompletedServiceResponse = new MessageViewState()
            {
                serviceID = serviceID,
                timestamp = Time.time,
                topic = lastCompletedServiceRequest.topic,
                message = response,
                label = $"{lastCompletedServiceRequest.topic} Service Response",
            };
        }

        void InitStyles()
        {
            // Define font styles
            m_LabelStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
                fontStyle = FontStyle.Bold,
                fixedWidth = 250
            };

            m_ConnectionArrowStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
                fontSize = 22,
                fontStyle = FontStyle.Bold,
                fixedWidth = 250
            };

            m_ContentStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 0, 5),
                normal = { textColor = Color.white },
                fixedWidth = 300
            };

            m_MessageStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 5, 5),
                normal = { textColor = Color.white },
                fixedWidth = 300,
                wordWrap = true
            };

            m_ScrollRect = new Rect();
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
                return Color.Lerp(mid, dark, elapsedTime / fadeToDarkDuration);
        }

        void OnGUI()
        {
            if (!isEnabled)
                return;

            // Initialize main HUD
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(300));

            // ROS IP Setup
            GUILayout.BeginHorizontal();
            Color baseColor = GUI.color;
            GUI.color = Color.white;
            GUI.Label(new Rect(4, 5, 25, 15), "I", m_ConnectionArrowStyle);
            GUI.color = GetConnectionColor(Time.realtimeSinceStartup - m_LastMessageReceivedRealtime);
            GUI.Label(new Rect(8, 6, 25, 15), "\u2190", m_ConnectionArrowStyle);
            GUI.color = GetConnectionColor(Time.realtimeSinceStartup - m_LastMessageSentRealtime);
            GUI.Label(new Rect(8, 0, 25, 15), "\u2192", m_ConnectionArrowStyle);
            GUI.color = baseColor;

#if ROS2
            string protocolName = "ROS2";
#else
            string protocolName = "ROS";
#endif

            GUILayout.Space(30);
            GUILayout.Label($"{protocolName} IP: ", m_LabelStyle, GUILayout.Width(100));

            if (!ROSConnection.instance.HasConnectionThread)
            {
                // if you've never run a build on this machine before, initialize the playerpref settings to the ones from the RosConnection
                if (!PlayerPrefs.HasKey(PlayerPrefsKey_ROS_IP))
                    SetIPPref(ROSConnection.instance.RosIPAddress);
                if (!PlayerPrefs.HasKey(PlayerPrefsKey_ROS_TCP_PORT))
                    SetPortPref(ROSConnection.instance.RosPort);

                // NB, here the user is editing the PlayerPrefs values, not the ones in the RosConnection.
                // (So that the hud remembers what IP you used last time you ran this build.)
                // The RosConnection receives the edited values when you click Connect.
                SetIPPref(GUILayout.TextField(RosIPAddressPref));
                SetPortPref(Convert.ToInt32(GUILayout.TextField(RosPortPref.ToString())));

                GUILayout.EndHorizontal();
                GUILayout.Label("(Not connected)");
                if (GUILayout.Button("Connect"))
                    ROSConnection.instance.Connect(RosIPAddressPref, RosPortPref);
            }
            else
            {
                GUILayout.Label(host, m_ContentStyle);
                GUILayout.EndHorizontal();
            }

            // Last message sent
            GUILayout.Label("Last Message Sent:", m_LabelStyle);
            GUILayout.Label(m_LastMessageSentMeta, m_ContentStyle);
            if (m_LastMessageSent != null)
                m_ViewSent = GUILayout.Toggle(m_ViewSent, "View contents");

            // Last message received
            GUILayout.Label("Last Message Received:", m_LabelStyle);
            GUILayout.Label(m_LastMessageReceivedMeta, m_ContentStyle);
            if (m_LastMessageReceived != null)
                m_ViewRecv = GUILayout.Toggle(m_ViewRecv, "View contents");

            GUILayout.Label($"{activeServices.Count} Active Service Requests:", m_LabelStyle);
            if (activeServices.Count > 0)
            {
                var dots = new String('.', (int)Time.time % 4);
                GUILayout.Label($"Waiting for service response{dots}", m_ContentStyle);
            }

            m_ViewSrvs = GUILayout.Toggle(m_ViewSrvs, "View services status");

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

            // Update length of scroll
            if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
                m_ScrollRect = GUILayoutUtility.GetLastRect();

            // Optionally show message contents
            float y = m_ScrollRect.yMax;
            if (m_ViewSent)
            {
                y = ShowMessage(m_LastMessageSent, y);
            }

            if (m_ViewRecv)
            {
                y = ShowMessage(m_LastMessageReceived, y);
            }

            if (m_ViewSrvs)
            {
                foreach (MessageViewState service in activeServices)
                {
                    y = ShowMessage(service, y, showElapsedTime: true);
                }

                if (lastCompletedServiceRequest != null && lastCompletedServiceResponse != null)
                {
                    y = ShowMessage(lastCompletedServiceRequest, y);
                    y = ShowMessage(lastCompletedServiceResponse, y);
                }
            }

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

        /// <summary>
        /// All the information necessary to display a message and remember its scroll position
        /// </summary>
        class MessageViewState
        {
            public string label;
            public int serviceID;
            public float timestamp;
            public string topic;
            public Message message;
            public Rect contentRect;
            public Vector2 scrollPosition;
        }

        /// <summary>
        /// Displays a MessageViewState
        /// </summary>
        /// <param name="msgView">The message view to draw</param>
        /// <param name="y">The Y position to draw at</param>
        /// <param name="showElapsedTime">Whether to add elapsed time to the title</param>
        /// <returns>The new Y position to draw at</returns>
        float ShowMessage(MessageViewState msgView, float y, bool showElapsedTime = false)
        {
            if (msgView == null)
                return y;

            // Start scrollviews
            float height = msgView.contentRect.height > 0 ? Mathf.Min(msgView.contentRect.height, 200) : 200;
            Rect panelRect = new Rect(0, y + 5, 325, height);
            msgView.scrollPosition = GUI.BeginScrollView(panelRect, msgView.scrollPosition, msgView.contentRect);

            GUILayout.BeginVertical("box");

            // Paste contents of message
            if (showElapsedTime)
                GUILayout.Label($"{msgView.label} ({Time.time - msgView.timestamp})", m_LabelStyle);
            else
                GUILayout.Label(msgView.label, m_LabelStyle);
            GUILayout.Label(msgView.message.ToString(), m_MessageStyle);

            GUILayout.EndVertical();
            GUI.EndScrollView();

            // Update size of internal rect view
            if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
                msgView.contentRect = GUILayoutUtility.GetLastRect();

            return panelRect.yMax;
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

        // void RegisterTopics(string[] topics, string[] messageNames)
        // List<Action<Dictionary<string, string>>>
        void RegisterTopics(Dictionary<string, string> callback)
        {
            foreach (var c in callback)
            {
                string topic = c.Key;
                string type = c.Value;
                if (!s_AllTopics.ContainsKey(topic))
                {
                    s_AllTopics.Add(topic, null);
                }

                Debug.Log($"hudpanel register topics: {topic}: {type}");
                s_MessageNamesByTopic[topic] = type;
            }
            // Debug.Log($"in register topics, lens are {topics.Length}, {messageNames.Length}");
            // for (int Idx = 0; Idx < topics.Length; ++Idx)
            // {
            //     string topic = topics[Idx];
            //     if (!s_AllTopics.ContainsKey(topic))
            //     {
            //         s_AllTopics.Add(topic, null);
            //     }
            //
            //     Debug.Log($"hudpanel register topics: {topic}: {messageNames[Idx]}");
            //     s_MessageNamesByTopic[topic] = messageNames[Idx];
            // }
            // //ResizeTopicsWindow();
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
                GUILayout.Label("UI", HUDPanel.m_ConnectionArrowStyle, GUILayout.Width(20));
                GUILayout.Label("Viz", HUDPanel.m_ConnectionArrowStyle);
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
