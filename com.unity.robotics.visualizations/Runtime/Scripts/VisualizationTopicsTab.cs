using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
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
            UI,
            Viz,
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
            m_TopicFilter = GUILayout.TextField(m_TopicFilter);

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Space(5);
            if (GUILayout.Button("2D", HudPanel.s_BoldStyle, GUILayout.Width(20)))
            {
                SetSortMode(m_SortMode == SortMode.UI ? SortMode.Normal : SortMode.UI);
            }
            GUILayout.Space(5);
            if (GUILayout.Button("3D", HudPanel.s_BoldStyle, GUILayout.Width(25)))
            {
                SetSortMode(m_SortMode == SortMode.Viz ? SortMode.Normal : SortMode.Viz);
            }
            if (GUILayout.Button("Topic", HudPanel.s_BoldStyle))
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
                var topic = topicState.Topic;
                if (!topic.Contains(m_TopicFilter))
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
                /*                case SortMode.UI:
                                    m_TopicsSorted.Sort(
                                        (VisualizationTopicsTabEntry a, VisualizationTopicsTabEntry b) =>
                                        {
                                            int primary = b.CanShowWindow.CompareTo(a.CanShowWindow);
                                            return (primary != 0) ? primary : b.IsVisualizingUI.CompareTo(a.IsVisualizingUI);
                                        }
                                    );
                                    break;
                                case SortMode.Viz:
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
                                    break;*/
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
                    var save = rule.CreateSaveState();
                    if (save != null)
                        topicRuleSaves.Add(save);
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
                RosTopicState topicState = m_Connection.GetOrCreateTopic(savedRule.Topic, savedRule.RosMessageName);
                VisualizationTopicsTabEntry vis = new VisualizationTopicsTabEntry(savedRule, topicState, m_FillTexture);
                m_Topics.Add(savedRule.Topic, vis);
            }
        }

        void OnApplicationQuit()
        {
            SaveLayout();
        }
    }
}
