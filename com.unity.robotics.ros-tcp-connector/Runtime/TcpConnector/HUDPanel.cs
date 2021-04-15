using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.MessageVisualizers;
using System;
using System.Collections.Generic;
using System.Reflection;
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
        static Dictionary<string, Type> s_MessageTypesByName;

        public static string GetMessageNameByTopic(string topic)
        {
            string rosMessageName;
            if (!s_MessageNamesByTopic.TryGetValue(topic, out rosMessageName))
            {
                return null;
            }

            return rosMessageName;
        }

        public static Type GetMessageClassByName(string rosMessageName)
        {
            if(s_MessageTypesByName == null)
            {
                s_MessageTypesByName = new Dictionary<string, Type>();
                Type baseMessageType = typeof(Message);
                foreach (Type classType in AppDomain.CurrentDomain.GetAssemblies().SelectMany(a => a.GetTypes()))
                {
                    if(baseMessageType.IsAssignableFrom(classType))
                    {
                        FieldInfo messageNameField = classType.GetField("k_RosMessageName");
                        if(messageNameField != null)
                            s_MessageTypesByName[(string)messageNameField.GetValue(null)] = classType;
                    }
                }
            }

            Type result;
            s_MessageTypesByName.TryGetValue(rosMessageName, out result);
            return result;
        }

        // ROS Message variables
        string m_RosConnectAddress = "";
        string m_UnityListenAddress = "Not listening";

        int m_NextServiceID = 101;
        int m_NextWindowID = 101;

        List<HUDVisualizationRule> m_ActiveWindows = new List<HUDVisualizationRule>();
        SortedList<string, HUDVisualizationRule> m_AllTopics = new SortedList<string, HUDVisualizationRule>();
        Dictionary<int, HUDVisualizationRule> m_PendingServiceRequests = new Dictionary<int, HUDVisualizationRule>();
        HUDVisualizationRule m_DraggingWindow;

        bool m_ShowingTopics;
        bool m_DidRequestTopics;
        Vector2 m_TopicMenuScrollPosition;
        Rect m_TopicMenuRect = new Rect(20, 70, 250, 200);
        string m_TopicFilter = "";

        bool m_ShowingTransforms;
        Vector2 m_TransformMenuScrollPosition;
        Rect m_TransformMenuRect = new Rect(20, 70, 250, 200);

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
        }

        /*
        public void SetLastMessageRaw(string topic, byte[] data)
        {
            HUDVisualizationRule rule;
            if (!m_AllTopics.TryGetValue(topic, out rule))
            {
                m_AllTopics.Add(topic, null);
            }
            if (rule != null)
                rule.SetMessageRaw(data, new MessageMetadata(topic, DateTime.Now));
        }*/

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

        public void OnStartMessageServer(string ip, int port)
        {
            m_UnityListenAddress = $"{ip}:{port}";
        }

        void OnGUI()
        {
            if (!IsEnabled)
                return;

            // Initialize main HUD
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(300));

            // ROS IP Setup
            GUILayout.BeginHorizontal();
            GUILayout.Label("ROS IP: ", s_BoldStyle, GUILayout.Width(100));
            GUILayout.Label(m_RosConnectAddress, s_IPStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Unity IP: ", s_BoldStyle, GUILayout.Width(100));
            GUILayout.Label(m_UnityListenAddress, s_IPStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Topics"))
            {
                m_ShowingTopics = !m_ShowingTopics;
                m_ShowingTransforms = false;
                if (m_ShowingTopics)
                {
                    ResizeTopicsWindow();
                    if (true)//!m_DidRequestTopics)
                    {
                        RequestTopics();
                    }
                }
            }
            if (GUILayout.Button("Transforms"))
            {
                m_ShowingTransforms = !m_ShowingTransforms;
                m_ShowingTopics = false;
            }
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();

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
                GUI.Window(0, m_TopicMenuRect, DrawTopicMenuContents, "", s_TopicMenuStyle);

                if (Event.current.type == EventType.MouseDown)
                    m_ShowingTopics = ShouldKeepMenuOpen();
            }

            // Transforms menu
            if(m_ShowingTransforms)
            {
                GUI.Window(0, m_TransformMenuRect, DrawTransformMenuContents, "", s_TopicMenuStyle);

                if (Event.current.type == EventType.MouseDown)
                    m_ShowingTransforms = ShouldKeepMenuOpen();
            }
        }

        void RequestTopics()
        {
            m_DidRequestTopics = true;
            ROSConnection.instance.GetTopicList(RegisterTopics);
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
        }

        void ResizeTopicsWindow()
        {
            m_TopicMenuRect.height = (m_AllTopics.Count + 2) * 25 + 5;
            if (m_TopicMenuRect.yMax > Screen.height)
                m_TopicMenuRect.yMax = Screen.height;
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
            GUILayout.Label("Scene", s_BoldStyle);
            GUILayout.EndHorizontal();

            m_TopicMenuScrollPosition = GUILayout.BeginScrollView(m_TopicMenuScrollPosition);
            int numTopicsShown = 0;
            foreach (KeyValuePair<string, HUDVisualizationRule> kv in m_AllTopics)
            {
                bool showWindow = false;
                bool showDrawing = false;
                string title = kv.Key;
                if (!title.Contains(m_TopicFilter))
                {
                    continue;
                }

                numTopicsShown++;
                HUDVisualizationRule rule = kv.Value;

                if (rule != null)
                {
                    showWindow = rule.ShowWindow;
                    showDrawing = rule.ShowDrawing;
                    title = rule.Topic;
                }
                bool hasWindow = showWindow;
                bool hasDrawing = showDrawing;

                GUILayout.BeginHorizontal();
                showWindow = GUILayout.Toggle(showWindow, "", GUILayout.Width(15));
                showDrawing = GUILayout.Toggle(showDrawing, "", GUILayout.Width(15));
                if (GUILayout.Button(title, GUI.skin.label, GUILayout.Width(170)))
                {
                    bool toggleOn = (!showWindow || !showDrawing);
                    showWindow = toggleOn;
                    showDrawing = toggleOn;
                }
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
                    m_ShowingTopics = ShouldKeepMenuOpen();
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

        const float k_TFNameWidth = 140;
        const float k_CheckboxWidth = 35;
        void DrawTransformMenuContents(int id)
        {
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

            foreach (TFStream stream in TFSystem.instance.GetTransforms())
            {
                GUI.changed = false;
                GUILayout.BeginHorizontal();
                GUILayout.Label(stream.Name, GUILayout.Width(k_TFNameWidth));
                stream.ShowAxes = GUILayout.Toggle(stream.ShowAxes, "", GUILayout.Width(k_CheckboxWidth));
                stream.ShowLink = GUILayout.Toggle(stream.ShowLink, "", GUILayout.Width(k_CheckboxWidth));
                stream.ShowName = GUILayout.Toggle(stream.ShowName, "", GUILayout.Width(k_CheckboxWidth));
                GUILayout.EndHorizontal();
                if(GUI.changed)
                {
                    TFSystem.UpdateVisualization(stream);
                }
                GUI.changed = false;
            }
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

        public bool ShouldKeepMenuOpen()
        {
            // If the user is holding shift, don't close the topic menu on selecting a topic
            return (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift));
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