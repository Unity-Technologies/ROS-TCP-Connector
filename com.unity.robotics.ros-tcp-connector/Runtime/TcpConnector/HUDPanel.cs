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
        // GUI variables
        public static GUIStyle headingStyle;
        public static GUIStyle ipStyle;
        public static GUIStyle contentStyle;
        public static GUIStyle messageStyle;
        public static GUIStyle boldStyle;
        public static GUIStyle boxStyle;
        public static GUIStyle topicMenuStyle;

        // ROS Message variables
        internal bool isEnabled;
        string rosConnectAddress = "";
        string unityListenAddress = "Not listening";

        List<TopicVisualizationRule> activeWindows = new List<TopicVisualizationRule>();
        TopicVisualizationRule draggingWindow;
        SortedList<string, TopicVisualizationRule> allTopics = new SortedList<string, TopicVisualizationRule>();
        Dictionary<int, TopicVisualizationRule> pendingServiceRequests = new Dictionary<int, TopicVisualizationRule>();
        GUIStyle topicEntryStyle;
        int nextServiceID = 101;
        int nextWindowID = 101;
        bool showingTopics;
        Vector2 topicsScrollPosition;
        string topicFilter = "";
        Rect topicsRect = new Rect(20,70,200,200);

        string LayoutFilePath => Path.Combine(Application.persistentDataPath, "RosHudLayout.json");

        void SaveLayout()
        {
            HUDLayoutSave saveState = new HUDLayoutSave { };
            saveState.AddRules(allTopics.Values);
            File.WriteAllText(LayoutFilePath, JsonUtility.ToJson(saveState));
        }

        void LoadLayout()
        {
            if(File.Exists(LayoutFilePath))
                LoadLayout(JsonUtility.FromJson<HUDLayoutSave>(File.ReadAllText(LayoutFilePath)));
        }

        void LoadLayout(HUDLayoutSave saveState)
        {
            activeWindows.Clear();
            foreach(HUDLayoutSave.TopicRuleSave rule in saveState.rules)
            {
                TopicVisualizationRule newRule = new TopicVisualizationRule(rule, this);
                nextWindowID++;
                allTopics[rule.topic] = newRule;
            }
        }

        void AddWindow(TopicVisualizationRule window)
        {
            activeWindows.Add(window);
        }

        void RemoveWindow(TopicVisualizationRule window)
        {
            activeWindows.Remove(window);
        }

        public int GetNextWindowID()
        {
            int result = nextWindowID;
            nextWindowID++;
            return result;
        }

        public void SetLastMessageSent(string topic, Message message)
        {
            TopicVisualizationRule rule;
            if(!allTopics.TryGetValue(topic, out rule))
            {
                allTopics.Add(topic, null);
            }
            if (rule != null)
                rule.SetMessage(message, new MessageMetadata(topic, DateTime.Now));
        }

        public void SetLastMessageReceived(string topic, Message message)
        {
            TopicVisualizationRule rule;
            if (!allTopics.TryGetValue(topic, out rule))
            {
                allTopics.Add(topic, null);
            }
            if (rule != null)
                rule.SetMessage(message, new MessageMetadata(topic, DateTime.Now));
        }

        public int AddServiceRequest(string topic, Message request)
        {
            int serviceID = nextServiceID;
            nextServiceID++;

            MessageMetadata meta = new MessageMetadata(topic, DateTime.Now);

            TopicVisualizationRule rule;
            if (!allTopics.TryGetValue(topic, out rule))
            {
                allTopics.Add(topic, null);
            }
            if (rule != null)
            {
                pendingServiceRequests.Add(serviceID, rule);
                rule.SetServiceRequest(request, new MessageMetadata(topic, DateTime.Now), serviceID);
            }
            return serviceID;
        }

        public void AddServiceResponse(int serviceID, Message response)
        {
            TopicVisualizationRule rule;
            if (!pendingServiceRequests.TryGetValue(serviceID, out rule))
                return; // don't know what happened there, but that's not a request I recognize

            pendingServiceRequests.Remove(serviceID);

            if(rule != null)
            {
                rule.SetServiceResponse(response, new MessageMetadata(rule.topic, DateTime.Now), serviceID);
            }
        }

        void Awake()
        {
            // Define font styles
            headingStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
                fontStyle = FontStyle.Bold,
                fixedWidth = 100
            };

            ipStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
            };

            boldStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                //padding = new RectOffset(10, 0, 0, 5),
                normal = { textColor = Color.white },
                fontStyle = FontStyle.Bold,
                fixedWidth = 300
            };

            contentStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                //padding = new RectOffset(10, 0, 0, 5),
                normal = { textColor = Color.white },
                fixedWidth = 300
            };

            messageStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                //padding = new RectOffset(10, 0, 5, 5),
                normal = { textColor = Color.white },
                fixedWidth = 300,
                wordWrap = true
            };

            LoadLayout();
        }

        private void OnApplicationQuit()
        {
            SaveLayout();
        }

        public void SetRosIP(string ip, int port)
        {
            rosConnectAddress = $"{ip}:{port}";
        }

        public void OnStartMessageServer(string ip, int port)
        {
            unityListenAddress = $"{ip}:{port}";
        }

        void OnGUI()
        {
            if (!isEnabled)
                return;

            if(topicMenuStyle == null)
            {
                topicMenuStyle = new GUIStyle
                {
                    stretchHeight = GUI.skin.box.stretchHeight,
                    stretchWidth = GUI.skin.box.stretchWidth,
                    padding = GUI.skin.box.padding,
                    border = GUI.skin.box.border,
                    normal = { background = GUI.skin.box.normal.background },
                };
            }

            if (boxStyle == null)
            {
                boxStyle = GUI.skin.GetStyle("box");
                boxStyle.fixedWidth = 300;
            }

            // Initialize main HUD
            GUILayout.BeginVertical(boxStyle);

            // ROS IP Setup
            GUILayout.BeginHorizontal();
            GUILayout.Label("ROS IP: ", headingStyle);
            GUILayout.Label(rosConnectAddress, ipStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Unity IP: ", headingStyle);
            GUILayout.Label(unityListenAddress, ipStyle);
            GUILayout.EndHorizontal();

            if(GUILayout.Button("Visualizations"))
            {
                showingTopics = true;
                topicsRect.height = (allTopics.Count+1) * 25;
                if (topicsRect.yMax > Screen.height)
                    topicsRect.yMax = Screen.height;
            }

            GUILayout.EndVertical();

            Event current = Event.current;
            if (current.type == EventType.MouseDown)
            {
                for (int Idx = activeWindows.Count - 1; Idx >= 0; --Idx)
                {
                    TopicVisualizationRule window = activeWindows[Idx];
                    if (activeWindows[Idx].TryDragWindow(current))
                    {
                        draggingWindow = activeWindows[Idx];
                        break;
                    }
                }
            }
            else if (current.type == EventType.MouseDrag && draggingWindow != null)
            {
                draggingWindow.UpdateDragWindow(current);
            }
            else if (current.type == EventType.MouseUp && draggingWindow != null)
            {
                draggingWindow.EndDragWindow();
                draggingWindow = null;
            }

            foreach (TopicVisualizationRule window in activeWindows)
            {
                window.DrawWindow();
            }

            if (showingTopics)
            {
                GUI.Window(0, topicsRect, DrawTopicWindowContents, "", topicMenuStyle);

                if (Event.current.type == EventType.MouseDown)
                    TryCloseTopics();
            }
        }

        void DrawTopicWindowContents(int id)
        {
            if(topicEntryStyle == null)
            {
                topicEntryStyle = new GUIStyle(GUI.skin.toggle);
            }
            topicFilter = GUILayout.TextField(topicFilter);

            GUILayout.BeginHorizontal();
            GUILayout.Label("UI", boldStyle, GUILayout.Width(20));
            GUILayout.Label("Scene", boldStyle);
            GUILayout.EndHorizontal();

            topicsScrollPosition = GUILayout.BeginScrollView(topicsScrollPosition);
            int numTopicsShown = 0;
            foreach (KeyValuePair<string, TopicVisualizationRule> kv in allTopics)
            {
                bool showWindow = false;
                bool showDrawing = false;
                string title = kv.Key;
                if(!title.Contains(topicFilter))
                {
                    continue;
                }

                numTopicsShown++;
                TopicVisualizationRule rule = kv.Value;

                if (rule != null)
                {
                    showWindow = rule.showWindow;
                    showDrawing = rule.showDrawing;
                    title = rule.topic;
                }
                bool hasWindow = showWindow;
                bool hasDrawing = showDrawing;

                GUILayout.BeginHorizontal();
                showWindow = GUILayout.Toggle(showWindow, "", GUILayout.Width(15));
                showDrawing = GUILayout.Toggle(showDrawing, "", GUILayout.Width(15));
                if (GUILayout.Button(title, GUI.skin.label, GUILayout.Width(100)))
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
                        rule = new TopicVisualizationRule(kv.Key, this);
                        allTopics[kv.Key] = rule;
                    }
                    rule.SetShowWindow(showWindow);
                    rule.SetShowDrawing(showDrawing);
                    TryCloseTopics();
                    break;
                }
            }
            GUILayout.EndScrollView();

            if(numTopicsShown == 0)
            {
                if(allTopics.Count == 0)
                    GUILayout.Label("No known topics yet");
                else
                    GUILayout.Label("No such topic!");
            }
        }

        public void TryCloseTopics()
        {
            if (!Input.GetKey(KeyCode.LeftShift) && !Input.GetKey(KeyCode.RightShift))
                showingTopics = false;
        }

        class TopicVisualizationRule
        {
            public string topic { get; private set; }
            public bool showWindow { get; private set; }
            public bool showDrawing { get; private set; }
            float drawingDuration;
            HUDPanel hud;
            IWindowContents contents;

            public Rect windowRect;
            int windowID;
            int serviceID;
            public Vector2 windowScrollPosition { get; set; }

            const float draggableSize = 8;
            Vector2 dragMouseOffset;
            bool draggingTitle;
            bool draggingLeft;
            bool draggingRight;
            bool draggingBottom;

            public TopicVisualizationRule(HUDLayoutSave.TopicRuleSave saveState, HUDPanel hud)
            {
                this.hud = hud;
                windowRect = saveState.rect;
                topic = saveState.topic;
                windowID = hud.GetNextWindowID();
                SetShowWindow(saveState.showWindow);
                SetShowDrawing(saveState.showDrawing);
            }

            public TopicVisualizationRule(string topic, HUDPanel hud)
            {
                this.hud = hud;
                windowRect = new Rect(50, 70, 200, 100);
                this.topic = topic;
                showWindow = false;
                showDrawing = false;
                windowID = hud.GetNextWindowID();
            }

            public HUDLayoutSave.TopicRuleSave CreateSaveState()
            {
                if (!showWindow && !showDrawing)
                    return null;

                return new HUDLayoutSave.TopicRuleSave {
                    rect = windowRect,
                    topic = topic,
                    showWindow = showWindow,
                    showDrawing = showDrawing,
                    drawingDuration = drawingDuration
                };
            }

            public void SetMessage(Message message, MessageMetadata meta)
            {
                if (contents != null)
                    contents.ShowDrawing(false);

                contents = new MessageWindowContents(this, message, meta);
                if(showDrawing)
                    contents.ShowDrawing(true);
            }

            public void SetServiceRequest(Message request, MessageMetadata requestMeta, int serviceID)
            {
                if (contents != null)
                    contents.ShowDrawing(false);

                this.serviceID = serviceID;
                contents = new ServiceWindowContents(this, request, requestMeta);
                if (showDrawing)
                    contents.ShowDrawing(true);
            }

            public void SetServiceResponse(Message response, MessageMetadata responseMeta, int serviceID)
            {
                // If this is not a response to the request we have, ignore it.
                // TODO: need more granular control over this
                if (this.serviceID != serviceID)
                    return;

                if (contents != null)
                    contents.ShowDrawing(false);

                ((ServiceWindowContents)contents).SetResponse(response, responseMeta);
                if (showDrawing)
                    contents.ShowDrawing(true);
            }

            public void DrawWindow()
            {
                if(contents != null)
                    contents.DrawWindow(windowID, windowRect);
            }

            public void SetShowDrawing(bool showDrawing)
            {
                bool hasDrawing = contents != null && contents.hasDrawing;
                this.showDrawing = showDrawing;
                if (showDrawing != hasDrawing && contents != null)
                {
                    contents.ShowDrawing(showDrawing);
                }
            }

            public void SetShowWindow(bool showWindow)
            {
                bool hasWindow = this.showWindow;
                this.showWindow = showWindow;
                if (showWindow != hasWindow)
                {
                    if (showWindow)
                    {
                        if(contents == null)
                            contents = new MessageWindowContents(this, null, new MessageMetadata(topic, DateTime.Now));
                        hud.AddWindow(this);
                    }
                    else
                    {
                        hud.RemoveWindow(this);
                    }
                }
            }

            public bool TryDragWindow(Event current)
            {
                Rect expandedWindowMenu = new Rect(windowRect.x - draggableSize, windowRect.y, windowRect.width + draggableSize * 2, windowRect.height + draggableSize);
                if (expandedWindowMenu.Contains(current.mousePosition))
                {
                    draggingTitle = current.mousePosition.y < windowRect.yMin + draggableSize * 2;
                    draggingLeft = current.mousePosition.x < windowRect.xMin + draggableSize;
                    draggingRight = current.mousePosition.x > windowRect.xMax - draggableSize;
                    draggingBottom = current.mousePosition.y > windowRect.yMax - draggableSize;
                }

                dragMouseOffset = current.mousePosition - new Vector2(windowRect.xMin, windowRect.yMin);
                return draggingTitle || draggingLeft || draggingRight || draggingBottom;
            }

            public void UpdateDragWindow(Event current)
            {
                if (draggingTitle)
                {
                    windowRect.x = current.mousePosition.x - dragMouseOffset.x;
                    windowRect.y = current.mousePosition.y - dragMouseOffset.y;
                }
                else
                {
                    if (draggingLeft)
                        windowRect.xMin = current.mousePosition.x;
                    if (draggingBottom)
                        windowRect.yMax = current.mousePosition.y;
                    if (draggingRight)
                        windowRect.xMax = current.mousePosition.x;
                }
            }

            public void EndDragWindow()
            {
                draggingTitle = draggingLeft = draggingRight = draggingBottom = false;
            }
        }

        interface IWindowContents
        {
            bool hasDrawing { get; }
            void ShowDrawing(bool show);
            void DrawWindow(int windowID, Rect windowRect);
        }

        class MessageWindowContents : IWindowContents
        {
            TopicVisualizationRule rule;
            Message message;
            MessageMetadata meta;
            public IVisualizer visualizerConfig;
            public object visualizerDrawing;
            public Action visualizerGUI;

            public bool hasDrawing => visualizerDrawing != null;

            public MessageWindowContents(TopicVisualizationRule rule, Message message, MessageMetadata meta)
            {
                this.rule = rule;
                this.message = message;
                this.meta = meta;
            }

            public void ShowDrawing(bool show)
            {
                if (show)
                {
                    if (visualizerConfig == null && message != null)
                        visualizerConfig = VisualizationRegister.GetVisualizer(message, meta);

                    if (visualizerConfig != null)
                        visualizerDrawing = visualizerConfig.CreateDrawing(message, meta);
                }
                else
                {
                    if(visualizerConfig != null && visualizerDrawing != null)
                        visualizerConfig.DeleteDrawing(visualizerDrawing);
                    visualizerDrawing = null;
                }
            }

            public void DrawWindow(int windowID, Rect windowRect)
            {
                GUI.Window(windowID, windowRect, DrawWindowContents, meta.Topic);
            }

            void DrawWindowContents(int id)
            {
                if (message == null)
                {
                    GUILayout.Label("Waiting for message...");
                }
                else
                {
                    if (visualizerConfig == null)
                        visualizerConfig = VisualizationRegister.GetVisualizer(message, meta);

                    if (visualizerGUI == null)
                        visualizerGUI = visualizerConfig.CreateGUI(message, meta, visualizerDrawing);

                    rule.windowScrollPosition = GUILayout.BeginScrollView(rule.windowScrollPosition);
                    visualizerGUI();
                    GUILayout.EndScrollView();
                }
            }
        }

        class ServiceWindowContents : IWindowContents
        {
            TopicVisualizationRule rule;
            Message request;
            MessageMetadata requestMeta;
            IVisualizer requestVisualizer;
            object requestDrawing;
            public Action requestGUI;

            Message response;
            MessageMetadata responseMeta;
            IVisualizer responseVisualizer;
            object responseDrawing;
            public Action responseGUI;

            public bool hasDrawing => requestDrawing != null || responseDrawing != null;

            public ServiceWindowContents(TopicVisualizationRule rule, Message request, MessageMetadata requestMeta)
            {
                this.rule = rule;
                this.request = request;
                this.requestMeta = requestMeta;
            }

            public void SetResponse(Message response, MessageMetadata responseMeta)
            {
                this.response = response;
                this.responseMeta = responseMeta;
            }

            public void ShowDrawing(bool show)
            {
                if (show)
                {
                    if (requestVisualizer == null && request != null)
                        requestVisualizer = VisualizationRegister.GetVisualizer(request, requestMeta);

                    if (responseVisualizer == null && response != null)
                        responseVisualizer = VisualizationRegister.GetVisualizer(response, responseMeta);

                    if (requestVisualizer != null && requestDrawing == null)
                        requestDrawing = requestVisualizer.CreateDrawing(request, requestMeta);

                    if (responseVisualizer != null && responseDrawing == null)
                        responseDrawing = responseVisualizer.CreateDrawing(response, responseMeta);
                }
                else
                {
                    if(requestVisualizer != null && requestDrawing != null)
                        requestVisualizer.DeleteDrawing(requestDrawing);
                    requestDrawing = null;

                    if (responseVisualizer != null && responseDrawing != null)
                        responseVisualizer.DeleteDrawing(responseDrawing);
                    responseDrawing = null;
                }
            }

            public void DrawWindow(int windowID, Rect windowRect)
            {
                GUI.Window(windowID, windowRect, DrawWindowContents, requestMeta.Topic);
            }

            void DrawWindowContents(int id)
            {
                rule.windowScrollPosition = GUILayout.BeginScrollView(rule.windowScrollPosition);

                if (request == null)
                {
                    GUILayout.Label("Waiting for request...");
                }
                else
                {
                    if (requestVisualizer == null)
                        requestVisualizer = VisualizationRegister.GetVisualizer(request, requestMeta);

                    if (requestGUI == null)
                        requestGUI = requestVisualizer.CreateGUI(request, requestMeta, requestDrawing);

                    requestGUI();
                }

                // horizontal line
                GUILayout.Label("", GUI.skin.horizontalSlider);

                if (response == null)
                {
                    GUILayout.Label("Waiting for response...");
                }
                else
                {
                    if (responseVisualizer == null)
                        responseVisualizer = VisualizationRegister.GetVisualizer(response, responseMeta);

                    if (responseGUI == null)
                        responseGUI = responseVisualizer.CreateGUI(response, responseMeta, responseDrawing);

                    responseGUI();
                }
                GUILayout.EndScrollView();
            }
        }

        class HUDLayoutSave
        {
            [Serializable]
            public class TopicRuleSave
            {
                public Rect rect;
                public string topic;
                public bool showWindow;
                public bool showDrawing;
                public float drawingDuration;
            }

            public TopicRuleSave[] rules;

            public void AddRules(IEnumerable<TopicVisualizationRule> rules)
            {
                List<TopicRuleSave> topicRuleSaves = new List<TopicRuleSave>();
                foreach (TopicVisualizationRule rule in rules)
                {
                    if (rule == null)
                        continue;
                    TopicRuleSave save = rule.CreateSaveState();
                    if (save != null)
                        topicRuleSaves.Add(save);
                }
                this.rules = topicRuleSaves.ToArray();
            }
        }
    }
}