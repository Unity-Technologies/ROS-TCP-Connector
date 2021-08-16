using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class VisualizationTopicsTab : MonoBehaviour, IHudTab
    {
        ROSConnection m_Connection;

        string IHudTab.Label => "Topics";

        string m_TopicFilter = "";

        Vector2 m_TopicMenuScrollPosition;

        string LayoutFilePath => System.IO.Path.Combine(Application.persistentDataPath, "RosHudLayout.json");

        public void Start()
        {
            m_Connection = ROSConnection.GetOrCreateInstance();
            HudPanel.RegisterTab(this, (int)HudTabOrdering.Topics);
            HudPanel.RegisterTab(new VisualizationLayoutTab(this), (int)HudTabOrdering.Layout);
            HudPanel.RegisterTab(new VisualizationMarkersTab(m_Connection), (int)HudTabOrdering.Markers);
            LoadLayout();
            m_Connection.ListenForTopics(OnNewTopic, notifyAllExistingTopics: true);
        }

        void OnNewTopic(RosTopicState state)
        {
            RosTopicVisualizationState.GetOrCreate(state);
        }


        void IHudTab.OnGUI(HudPanel hud)
        {
            m_Connection.RefreshTopicsList();

            GUILayout.BeginHorizontal();
            m_TopicFilter = GUILayout.TextField(m_TopicFilter);

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("UI", HudPanel.s_BoldStyle, GUILayout.Width(20));
            GUILayout.Label("Viz", HudPanel.s_BoldStyle);
            GUILayout.EndHorizontal();

            m_TopicMenuScrollPosition = GUILayout.BeginScrollView(m_TopicMenuScrollPosition);
            var numTopicsShown = 0;
            foreach (RosTopicVisualizationState topicState in RosTopicVisualizationState.AllTopics)
            {
                var topic = topicState.Topic;
                if (!topic.Contains(m_TopicFilter))
                    continue;

                numTopicsShown++;
                topicState.DrawGUILine();
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

        public void OnSelected() { }
        public void OnDeselected() { }

        class HUDLayoutSave
        {
            public RosTopicVisualizationState.SaveState[] Rules;

            public void AddRules(IEnumerable<RosTopicVisualizationState> rules)
            {
                var topicRuleSaves = new List<RosTopicVisualizationState.SaveState>();
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
            saveState.AddRules(RosTopicVisualizationState.AllTopics);
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
                RosTopicVisualizationState.Load(savedRule, m_Connection);
            }
        }

        void OnApplicationQuit()
        {
            SaveLayout();
        }
    }
}
