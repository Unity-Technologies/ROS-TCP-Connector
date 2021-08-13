using System.IO;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEditor;

namespace Unity.Robotics.MessageVisualizers
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
                m_LayoutPath = EditorUtility.SaveFilePanel("Save Visualizations Settings", "", "RosHudLayout", "json");
                m_Topics.SaveLayout(m_LayoutPath);
            }
            if (GUILayout.Button("Import layout"))
            {
                m_LayoutPath = EditorUtility.OpenFilePanel("Select Visualizations Settings", "", "json");
                m_Topics.LoadLayout(m_LayoutPath);
            }
            GUILayout.EndHorizontal();
        }

        void IHudTab.OnSelected() { }
        void IHudTab.OnDeselected() { }
    }
}
