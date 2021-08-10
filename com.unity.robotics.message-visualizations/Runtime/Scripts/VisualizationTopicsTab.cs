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
            HudPanel.RegisterTab(this, (int)ROSConnection.HudTabIndices.Topics);
            HudPanel.RegisterTab(new VisualizerLayoutTab(this), (int)ROSConnection.HudTabIndices.Layout);
            m_Connection = ROSConnection.GetOrCreateInstance();
            m_Connection.ListenForTopics(OnNewTopic, notifyAllExistingTopics: true);
            LoadLayout();
        }

        List<VisualizationTopicState> m_Topics = new List<VisualizationTopicState>();

        void OnNewTopic(RosTopicState state)
        {
            m_Topics.Add(new VisualizationTopicState(state));
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
            foreach (VisualizationTopicState topicState in m_Topics)
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

        //============ HUD logic =================
        class HUDLayoutSave
        {
            public VisualizationTopicState.SaveState[] Rules;

            public void AddRules(IEnumerable<VisualizationTopicState> rules)
            {
                var topicRuleSaves = new List<VisualizationTopicState.SaveState>();
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
            saveState.AddRules(m_Topics);
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
                m_Topics.Add(new VisualizationTopicState(savedRule, m_Connection));
            }
        }

        void OnApplicationQuit()
        {
            SaveLayout();
        }
    }
}
