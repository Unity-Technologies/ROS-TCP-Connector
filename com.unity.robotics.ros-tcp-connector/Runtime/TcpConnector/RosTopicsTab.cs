using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class RosTopicsTab : IHudTab
    {
        ROSConnection m_Connection;

        public RosTopicsTab(ROSConnection connection)
        {
            m_Connection = connection;
        }

        string m_TopicFilter = "";

        Vector2 m_TopicMenuScrollPosition;
        string IHudTab.Label => "Topics";

        public void OnSelected() { }
        public void OnDeselected() { }

        public void OnGUI(HudPanel hud)
        {
            m_Connection.RefreshTopicsList();

            GUILayout.BeginHorizontal();
            m_TopicFilter = GUILayout.TextField(m_TopicFilter);

            /*
            if (m_TopicFilter != "" && !m_Connection.AllTopics.Contains(t=>t.Topic == m_TopicFilter))
            {
                if (GUILayout.Button($"Subscribe to \"{m_TopicFilter}\""))
                {
                    var state = new TopicVisualizationState(m_TopicFilter, GetMessageNameByTopic(m_TopicFilter), hud);
                    state.SetShowWindow(true);
                    state.SetShowDrawing(true);
                    s_AllTopics.Add(m_TopicFilter, state);
                }
            }*/

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("UI", HudPanel.s_BoldStyle, GUILayout.Width(20));
            GUILayout.Label("Viz", HudPanel.s_BoldStyle);
            GUILayout.EndHorizontal();

            m_TopicMenuScrollPosition = GUILayout.BeginScrollView(m_TopicMenuScrollPosition);
            var numTopicsShown = 0;
            foreach (RosTopicState topicState in m_Connection.AllTopics)
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
    }
}
