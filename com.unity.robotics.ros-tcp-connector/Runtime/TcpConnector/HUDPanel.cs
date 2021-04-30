using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.MessageVisualizers;
using System;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.IO;

namespace Unity.Robotics.ROSTCPConnector
{
    public class HUDPanel : MonoBehaviour
    {
        public bool IsEnabled { get; set; }

        // GUI variables
        public static GUIStyle s_IPStyle;
        public static GUIStyle s_BoldStyle;
        public static GUIStyle s_TopicMenuStyle;
        static Dictionary<string, string> s_MessageNamesByTopic = new Dictionary<string, string>();

        public static string GetMessageNameByTopic(string topic)
        {
            string rosMessageName;
            if (!s_MessageNamesByTopic.TryGetValue(topic, out rosMessageName))
            {
                return null;
            }

            return rosMessageName;
        }

        // ROS Message variables
        string m_RosConnectAddress = "";

        int m_NextServiceID = 101;
        int m_NextWindowID = 101;

        List<HUDVisualizationRule> m_ActiveWindows = new List<HUDVisualizationRule>();
        SortedList<string, HUDVisualizationRule> m_AllTopics = new SortedList<string, HUDVisualizationRule>();
        Dictionary<string, IVisualizer> m_TopicVisualizers = new Dictionary<string, IVisualizer>();
        Dictionary<int, HUDVisualizationRule> m_PendingServiceRequests = new Dictionary<int, HUDVisualizationRule>();
        HUDVisualizationRule m_DraggingWindow;

        bool m_ShowingTopics;
        bool m_DidRequestTopics;
        Vector2 m_TopicMenuScrollPosition;
        Rect m_TopicMenuRect = new Rect(0, 70, 300, 200);
        string m_TopicFilter = "";

        bool m_ShowingTransforms;
        Vector2 m_TransformMenuScrollPosition;
        Rect m_TransformMenuRect = new Rect(0, 70, 300, 200);

        float m_MessageOutLastRealtime;
        float m_MessageInLastRealtime;
        float m_LastTopicsRequestRealtime = -1;
        const float k_TimeBetweenTopicsUpdates = 5.0f;

        string LayoutFilePath => Path.Combine(Application.persistentDataPath, "RosHudLayout.json");

        void SaveLayout()
        {
            HUDLayoutSave saveState = new HUDLayoutSave { };
            saveState.AddRules(m_AllTopics.Values);
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
            foreach (HUDVisualizationRule.SaveState savedRule in saveState.Rules)
            {
                m_AllTopics[savedRule.Topic] = new HUDVisualizationRule(savedRule, this);
            }
        }

        public void AddWindow(HUDVisualizationRule window)
        {
            m_ActiveWindows.Add(window);
        }

        public void RemoveWindow(HUDVisualizationRule window)
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
            HUDVisualizationRule rule;
            if (!m_AllTopics.TryGetValue(topic, out rule))
            {
                m_AllTopics.Add(topic, null);
            }
            if (rule != null)
                rule.SetMessage(message, new MessageMetadata(topic, DateTime.Now));
            m_MessageOutLastRealtime = Time.realtimeSinceStartup;
        }

        public void SetLastMessageReceived(string topic, Message message)
        {
            HUDVisualizationRule rule;
            if (!m_AllTopics.TryGetValue(topic, out rule))
            {
                m_AllTopics.Add(topic, null);
            }
            if (rule != null)
                rule.SetMessage(message, new MessageMetadata(topic, DateTime.Now));
            m_MessageInLastRealtime = Time.realtimeSinceStartup;
        }

        public int AddServiceRequest(string topic, Message request)
        {
            int serviceID = m_NextServiceID;
            m_NextServiceID++;

            MessageMetadata meta = new MessageMetadata(topic, DateTime.Now);

            HUDVisualizationRule rule;
            if (!m_AllTopics.TryGetValue(topic, out rule))
            {
                m_AllTopics.Add(topic, null);
            }
            if (rule != null)
            {
                m_PendingServiceRequests.Add(serviceID, rule);
                rule.SetServiceRequest(request, new MessageMetadata(topic, DateTime.Now), serviceID);
            }
            return serviceID;
        }

        public void AddServiceResponse(int serviceID, Message response)
        {
            HUDVisualizationRule rule;
            if (!m_PendingServiceRequests.TryGetValue(serviceID, out rule))
                return; // don't know what happened there, but that's not a request I recognize

            m_PendingServiceRequests.Remove(serviceID);

            if (rule != null)
            {
                rule.SetServiceResponse(response, new MessageMetadata(rule.Topic, DateTime.Now), serviceID);
            }
        }

        void Awake()
        {
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

        void OnGUI()
        {
            if (!IsEnabled)
                return;

            // Initialize main HUD
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(300));

            // ROS IP Setup
            GUILayout.BeginHorizontal();
            Color baseColor = GUI.color;
            GUI.color = ROSConnection.instance.HasConnectionThread? (Time.realtimeSinceStartup - m_MessageOutLastRealtime) > 0.03f? Color.red: Color.green: Color.gray;
            GUILayout.Label("*", s_BoldStyle, GUILayout.Width(10));
            GUI.color = ROSConnection.instance.HasConnectionThread ? (Time.realtimeSinceStartup - m_MessageInLastRealtime) > 0.03f ? Color.red : Color.green: Color.gray;
            GUILayout.Label("*", s_BoldStyle, GUILayout.Width(10));
            GUI.color = baseColor;
            GUILayout.Label("ROS IP: ", s_BoldStyle, GUILayout.Width(100));
            GUILayout.Label(m_RosConnectAddress, s_IPStyle);
            GUILayout.EndHorizontal();

            if (!ROSConnection.instance.HasConnectionThread)
            {
                GUILayout.Label("(Not connected)");
                if (GUILayout.Button("Connect"))
                    ROSConnection.instance.Connect();
            }

            GUILayout.BeginHorizontal();
            m_ShowingTopics = GUILayout.Toggle(m_ShowingTopics, "Topics", GUI.skin.button);
            if(m_ShowingTopics)
            {
                m_ShowingTransforms = false;
                ResizeTopicsWindow();
                RequestTopics();
            }
            m_ShowingTransforms = GUILayout.Toggle(m_ShowingTransforms, "Transforms", GUI.skin.button);
            if(m_ShowingTransforms)
            {
                m_ShowingTopics = false;
            }
            GUILayout.EndHorizontal();

            // Draggable windows
            Event current = Event.current;
            if (current.type == EventType.MouseDown)
            {
                for (int Idx = m_ActiveWindows.Count - 1; Idx >= 0; --Idx)
                {
                    HUDVisualizationRule window = m_ActiveWindows[Idx];
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

            foreach (HUDVisualizationRule window in m_ActiveWindows)
            {
                window.DrawWindow();
            }

            if (s_TopicMenuStyle == null)
            {
                s_TopicMenuStyle = new GUIStyle
                {
                    stretchHeight = GUI.skin.box.stretchHeight,
                    stretchWidth = GUI.skin.box.stretchWidth,
                    padding = GUI.skin.box.padding,
                    border = GUI.skin.box.border,
                    normal = { background = GUI.skin.box.normal.background },
                };
            }

            // Topics menu
            if (m_ShowingTopics)
            {
                DrawTopicMenuContents(0);
            }

            // Transforms menu
            if(m_ShowingTransforms)
            {
                DrawTransformMenuContents(0);
            }
            GUILayout.EndVertical();
        }

        void RequestTopics()
        {
            if (m_LastTopicsRequestRealtime == -1 || (Time.realtimeSinceStartup - m_LastTopicsRequestRealtime) > k_TimeBetweenTopicsUpdates)
            {
                ROSConnection.instance.GetTopicList(RegisterTopics);
                m_LastTopicsRequestRealtime = Time.realtimeSinceStartup;
            }
        }

        void RegisterTopics(string[] topics, string[] messageNames)
        {
            for(int Idx = 0; Idx < topics.Length; ++Idx)
            {
                string topic = topics[Idx];
                if (!m_AllTopics.ContainsKey(topic))
                    m_AllTopics.Add(topic, null);
                s_MessageNamesByTopic[topic] = messageNames[Idx];
            }
            ResizeTopicsWindow();
            m_TopicVisualizers.Clear(); // update to the newest message types
        }

        void ResizeTopicsWindow()
        {
            m_TopicMenuRect.height = (m_AllTopics.Count + 2) * 25 + 5;
            if (m_TopicMenuRect.yMax > Screen.height)
                m_TopicMenuRect.yMax = Screen.height;
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
            foreach(HUDVisualizationRule window in m_ActiveWindows)
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
            foreach (HUDVisualizationRule window in m_ActiveWindows)
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

        void DrawTopicMenuContents(int id)
        {
            GUILayout.BeginHorizontal();
            m_TopicFilter = GUILayout.TextField(m_TopicFilter);

            if (m_TopicFilter != "" && !m_AllTopics.ContainsKey(m_TopicFilter))
            {
                if (GUILayout.Button($"Subscribe to \"{m_TopicFilter}\""))
                {
                    HUDVisualizationRule rule = new HUDVisualizationRule(m_TopicFilter, GetMessageNameByTopic(m_TopicFilter), this);
                    rule.SetShowWindow(true);
                    rule.SetShowDrawing(true);
                    m_AllTopics.Add(m_TopicFilter, rule);
                }
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("UI", s_BoldStyle, GUILayout.Width(20));
            GUILayout.Label("Viz", s_BoldStyle);
            GUILayout.EndHorizontal();

            m_TopicMenuScrollPosition = GUILayout.BeginScrollView(m_TopicMenuScrollPosition);
            int numTopicsShown = 0;
            foreach (KeyValuePair<string, HUDVisualizationRule> kv in m_AllTopics)
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
                HUDVisualizationRule rule = kv.Value;

                if (rule != null)
                {
                    showWindow = rule.ShowWindow;
                    showDrawing = rule.ShowDrawing;
                    title = rule.Topic;
                }

                IVisualizer visualizer = GetVisualizer(kv.Key);
                canShowWindow = visualizer != null;
                canShowDrawing = visualizer != null? visualizer.CanShowDrawing: false;

                bool hasWindow = showWindow;
                bool hasDrawing = showDrawing;

                GUILayout.BeginHorizontal();
                if(hasWindow || canShowWindow)
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
                    else if(!canShowDrawing)
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
                    if (rule == null)
                    {
                        rule = new HUDVisualizationRule(kv.Key, GetMessageNameByTopic(kv.Key), this);
                        m_AllTopics[kv.Key] = rule;
                    }
                    rule.SetShowWindow(showWindow);
                    rule.SetShowDrawing(showDrawing);
                    break;
                }
            }
            GUILayout.EndScrollView();

            if (numTopicsShown == 0)
            {
                if (m_AllTopics.Count == 0)
                    GUILayout.Label("No topics registered");
                else
                    GUILayout.Label($"No topics named \"{m_TopicFilter}\"!");
            }
        }

        IVisualizer GetVisualizer(string topic)
        {
            IVisualizer result;
            if (m_TopicVisualizers.TryGetValue(topic, out result))
                return result;

            string rosMessageName = GetMessageNameByTopic(topic);
            result = VisualizationRegistry.GetVisualizer(topic, rosMessageName);
            m_TopicVisualizers[topic] = result;
            return result;
        }

        const float k_IndentWidth = 10;
        const float k_TFNameWidth = 136;
        const float k_CheckboxWidth = 35;
        void DrawTransformMenuContents(int id)
        {
            m_TransformMenuScrollPosition = GUILayout.BeginScrollView(m_TransformMenuScrollPosition);

            GUI.changed = false;
            GUILayout.BeginHorizontal(GUILayout.Height(20));
            GUILayout.Label("", GUILayout.Width(k_TFNameWidth));
            TFSystem.instance.ShowTFAxesDefault = DrawTFHeaderCheckbox(TFSystem.instance.ShowTFAxesDefault, "Axes",
                (stream, check) => stream.ShowAxes = check);
            TFSystem.instance.ShowTFLinksDefault = DrawTFHeaderCheckbox(TFSystem.instance.ShowTFLinksDefault, "Link",
                (stream, check) => stream.ShowLink = check);
            TFSystem.instance.ShowTFNamesDefault = DrawTFHeaderCheckbox(TFSystem.instance.ShowTFNamesDefault, "Lbl",
                (stream, check) => stream.ShowName = check);
            GUILayout.EndHorizontal();
            bool globalChange = GUI.changed;

            // draw the root objects
            foreach (TFStream stream in TFSystem.instance.GetTransforms())
            {
                if(stream.Parent == null)
                    DrawTFStreamHierarchy(stream, 0, globalChange);
            }

            GUILayout.EndScrollView();
        }

        void DrawTFStreamHierarchy(TFStream stream, int indent, bool globalChange)
        {
            GUI.changed = false;
            GUILayout.BeginHorizontal();
            GUILayout.Space(indent * k_IndentWidth);
            GUILayout.Label(stream.Name, GUILayout.Width(k_TFNameWidth - indent * k_IndentWidth));
            stream.ShowAxes = GUILayout.Toggle(stream.ShowAxes, "", GUILayout.Width(k_CheckboxWidth));
            stream.ShowLink = GUILayout.Toggle(stream.ShowLink, "", GUILayout.Width(k_CheckboxWidth));
            stream.ShowName = GUILayout.Toggle(stream.ShowName, "", GUILayout.Width(k_CheckboxWidth));
            GUILayout.EndHorizontal();
            if (GUI.changed || globalChange)
            {
                TFSystem.UpdateVisualization(stream);
            }

            foreach (TFStream child in stream.Children)
                DrawTFStreamHierarchy(child, indent+1, globalChange);
        }

        bool DrawTFHeaderCheckbox(bool wasChecked, string label, Action<TFStream, bool> setter)
        {
            bool result = GUILayout.Toggle(wasChecked, "", GUILayout.Width(k_CheckboxWidth));
            
            Rect checkbox = GUILayoutUtility.GetLastRect();
            Vector2 textSize = GUI.skin.label.CalcSize(new GUIContent(label));
            Rect labelRect = new Rect(checkbox.xMin - textSize.x, checkbox.yMin, textSize.x, checkbox.height);
            GUI.Label(labelRect, label);

            if (wasChecked != result)
            {
                foreach(TFStream stream in TFSystem.instance.GetTransforms())
                {
                    setter(stream, result);
                }
            }
            return result;
        }

        class HUDLayoutSave
        {
            public HUDVisualizationRule.SaveState[] Rules;

            public void AddRules(IEnumerable<HUDVisualizationRule> rules)
            {
                List<HUDVisualizationRule.SaveState> topicRuleSaves = new List<HUDVisualizationRule.SaveState>();
                foreach (HUDVisualizationRule rule in rules)
                {
                    if (rule == null)
                        continue;
                    HUDVisualizationRule.SaveState save = rule.CreateSaveState();
                    if (save != null)
                        topicRuleSaves.Add(save);
                }
                this.Rules = topicRuleSaves.ToArray();
            }
        }
    }
}