using System.IO;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEditor;

namespace Unity.Robotics.Visualizations
{
    public class VisualizationLayoutTab : IHudTab
    {
        string IHudTab.Label => "Layout";
        string m_LayoutPath;
        VisualizationTopicsTab m_Topics;

        public VisualizationLayoutTab(VisualizationTopicsTab topics)
        {
            m_Topics = topics;
        }

        void IHudTab.OnGUI(HudPanel hud)
        {
            // Save/Load layout files
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Export layout"))
            {
#if UNITY_EDITOR
                m_LayoutPath = EditorUtility.SaveFilePanel("Save Visualizations Settings", "", "RosHudLayout", "json");
#endif
                m_Topics.SaveLayout(m_LayoutPath);
            }
            if (GUILayout.Button("Import layout"))
            {
#if UNITY_EDITOR
                m_LayoutPath = EditorUtility.OpenFilePanel("Select Visualizations Settings", "", "json");
#endif
                m_Topics.LoadLayout(m_LayoutPath);
            }
            GUILayout.EndHorizontal();
        }

        void IHudTab.OnSelected() { }
        void IHudTab.OnDeselected() { }
    }
}
