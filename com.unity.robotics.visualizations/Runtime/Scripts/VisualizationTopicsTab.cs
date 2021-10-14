using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class VisualizationTopicsTab : MonoBehaviour, IHudTab
    {
        ROSConnection m_Connection;
        Dictionary<string, VisualizationTopicsTabEntry> m_Topics = new Dictionary<string, VisualizationTopicsTabEntry>();
        List<VisualizationTopicsTabEntry> m_TopicsSorted;

        string IHudTab.Label => "Topics";

        string m_TopicFilter = "";

        Vector2 m_TopicMenuScrollPosition;

        string LayoutFilePath => System.IO.Path.Combine(Application.persistentDataPath, "RosHudLayout.json");

        enum SortMode
        {
            Normal,
            v2D,
            v3D,
            Topic,
            TopicDescending,
        }
        SortMode m_SortMode;
        Texture2D m_FillTexture;

        public void Start()
        {
            m_FillTexture = VisualizationUtils.MakeTexture(16, 16, new Color(0.125f, 0.19f, 0.25f));

            m_Connection = ROSConnection.GetOrCreateInstance();
            HudPanel.RegisterTab(this, (int)HudTabOrdering.Topics);
            HudPanel.RegisterTab(new VisualizationLayoutTab(this), (int)HudTabOrdering.Layout);
            LoadLayout();
            m_Connection.ListenForTopics(OnNewTopic, notifyAllExistingTopics: true);
        }

        void OnNewTopic(RosTopicState state)
        {
            VisualizationTopicsTabEntry vis;
            if (!m_Topics.TryGetValue(state.Topic, out vis))
            {
                vis = new VisualizationTopicsTabEntry(state, m_FillTexture);
                m_Topics.Add(state.Topic, vis);
                m_TopicsSorted = null;
            }
        }


        void IHudTab.OnGUI(HudPanel hud)
        {
            m_Connection.RefreshTopicsList();

            GUILayout.BeginHorizontal();
            bool showPrompt = (GUI.GetNameOfFocusedControl() != "topic_filter" && m_TopicFilter == "");
            GUI.SetNextControlName("topic_filter");
            if (showPrompt)
            {
                Color oldCol = GUI.color;
                GUI.color = new Color(oldCol.r, oldCol.g, oldCol.b, 0.5f);
                GUILayout.TextField("(Type here to filter topics)");
                GUI.color = oldCol;
            }
            else
            {
                m_TopicFilter = GUILayout.TextField(m_TopicFilter).ToLower();
            }

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (m_SortMode != SortMode.v2D)
                GUILayout.Space(5);
            string label2D = m_SortMode == SortMode.v2D ? "\u25BC2D" : "2D";
            if (GUILayout.Button(label2D, HudPanel.s_BoldStyle, GUILayout.Width(20)))
            {
                SetSortMode(m_SortMode == SortMode.v2D ? SortMode.Normal : SortMode.v2D);
            }
            if (m_SortMode == SortMode.v2D)
                GUILayout.Space(5);
            if (m_SortMode != SortMode.v3D)
                GUILayout.Space(5);
            string label3D = m_SortMode == SortMode.v3D ? "\u25BC3D" : "3D";
            if (GUILayout.Button(label3D, HudPanel.s_BoldStyle, GUILayout.Width(25)))
            {
                SetSortMode(m_SortMode == SortMode.v3D ? SortMode.Normal : SortMode.v3D);
            }
            if (m_SortMode == SortMode.v3D)
                GUILayout.Space(5);
            string labelTopic = m_SortMode == SortMode.Topic ? "\u25BCTopic" : m_SortMode == SortMode.TopicDescending ? "\u25B2Topic" : "Topic";
            if (GUILayout.Button(labelTopic, HudPanel.s_BoldStyle))
            {
                if (m_SortMode == SortMode.TopicDescending)
                    SetSortMode(SortMode.Normal);
                else if (m_SortMode == SortMode.Topic)
                    SetSortMode(SortMode.TopicDescending);
                else
                    SetSortMode(SortMode.Topic);
            }
            GUILayout.EndHorizontal();

            m_TopicMenuScrollPosition = GUILayout.BeginScrollView(m_TopicMenuScrollPosition);
            var numTopicsShown = 0;

            if (m_TopicsSorted == null)
                SortTopics();

            foreach (VisualizationTopicsTabEntry topicState in m_TopicsSorted)
            {
                if (m_TopicFilter != "" && !topicState.Topic.ToLower().Contains(m_TopicFilter) && !topicState.RosMessageName.ToLower().Contains(m_TopicFilter))
                    continue;

                numTopicsShown++;
                topicState.DrawGUI();
            }

            GUILayout.EndScrollView();

            if (numTopicsShown == 0)
            {
                if (!m_Connection.AllTopics.Any())
                    GUILayout.Label("No topics registered");
                else
                    GUILayout.Label($"No topics named \"{m_TopicFilter}\"!");
            }
        }

        void SetSortMode(SortMode sortMode)
        {
            m_SortMode = sortMode;
            m_TopicsSorted = null;
        }

        void SortTopics()
        {
            m_TopicsSorted = m_Topics.Values.ToList();
            switch (m_SortMode)
            {
                case SortMode.v2D:
                    m_TopicsSorted.Sort(
                        (VisualizationTopicsTabEntry a, VisualizationTopicsTabEntry b) =>
                        {
                            int primary = b.CanShowWindow.CompareTo(a.CanShowWindow);
                            return (primary != 0) ? primary : b.IsVisualizingUI.CompareTo(a.IsVisualizingUI);
                        }
                    );
                    break;
                case SortMode.v3D:
                    m_TopicsSorted.Sort(
                        (VisualizationTopicsTabEntry a, VisualizationTopicsTabEntry b) =>
                        {
                            int primary = b.CanShowWindow.CompareTo(a.CanShowWindow);
                            if (primary != 0)
                                return primary;
                            int secondary = b.CanShowDrawing.CompareTo(a.CanShowDrawing);
                            return (secondary != 0) ? secondary : b.IsVisualizingDrawing.CompareTo(a.IsVisualizingDrawing);
                        }
                    );
                    break;
                case SortMode.Topic:
                    m_TopicsSorted.Sort(
                        (VisualizationTopicsTabEntry a, VisualizationTopicsTabEntry b) => a.Topic.CompareTo(b.Topic)
                    );
                    break;
                case SortMode.TopicDescending:
                    m_TopicsSorted.Sort(
                        (VisualizationTopicsTabEntry a, VisualizationTopicsTabEntry b) => b.Topic.CompareTo(a.Topic)
                    );
                    break;
                default:
                    break;
            }
        }

        public void OnSelected() { }
        public void OnDeselected() { }

        class HUDLayoutSave
        {
            public VisualizationTopicsTabEntry.SaveState[] Rules;

            public void AddRules(IEnumerable<VisualizationTopicsTabEntry> rules)
            {
                var topicRuleSaves = new List<VisualizationTopicsTabEntry.SaveState>();
                foreach (var rule in rules)
                {
                    if (rule == null)
                        continue;
                    rule.AddSaveStates(topicRuleSaves);
                }

                Rules = topicRuleSaves.ToArray();
            }
        }

        public void SaveLayout(string path = "")
        {
            // Print filepath if saving to user-input path; default to persistentDataPath
            if (path.Length > 0)
            {
                Debug.Log($"Saved visualizations layout to {path}");
            }
            else
            {
                path = LayoutFilePath;
            }

            HUDLayoutSave saveState = new HUDLayoutSave { };
            saveState.AddRules(m_Topics.Values);
            System.IO.File.WriteAllText(path, JsonUtility.ToJson(saveState));
        }

        public void LoadLayout(string path = "")
        {
            if (path.Length > 0)
            {
                Debug.Log($"Loaded visualizations layout from {path}");
            }
            else
            {
                path = LayoutFilePath;
            }

            if (System.IO.File.Exists(path))
            {
                LoadLayout(JsonUtility.FromJson<HUDLayoutSave>(System.IO.File.ReadAllText(path)));
            }
        }

        void LoadLayout(HUDLayoutSave saveState)
        {
            foreach (var savedRule in saveState.Rules)
            {
                RosTopicState topicState = m_Connection.GetOrCreateTopic(savedRule.Topic, savedRule.RosMessageName, savedRule.IsService);
                VisualizationTopicsTabEntry vis;
                if (!m_Topics.TryGetValue(savedRule.Topic, out vis))
                {
                    vis = new VisualizationTopicsTabEntry(topicState, m_FillTexture);
                    m_Topics.Add(savedRule.Topic, vis);
                }

                vis.LoadSaveState(savedRule);
            }
        }

        void OnApplicationQuit()
        {
            SaveLayout();
        }
    }
}
