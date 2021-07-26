using System.IO;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEditor;

namespace Unity.Robotics.ROSTCPConnector
{
    public class VisualizerLayoutTab : IHudTab
    {
        string IHudTab.Label => "Layout";
        string m_LayoutPath;
        ROSConnection m_Connection;

        public VisualizerLayoutTab(ROSConnection connection)
        {
            m_Connection = connection;
        }

        void IHudTab.OnGUI(HudPanel hud)
        {
            // Save/Load layout files
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Export layout"))
            {
                m_LayoutPath = EditorUtility.SaveFilePanel("Save Visualizations Settings", "", "RosHudLayout", "json");
                m_Connection.SaveLayout(m_LayoutPath);
            }
            if (GUILayout.Button("Import layout"))
            {
                m_LayoutPath = EditorUtility.OpenFilePanel("Select Visualizations Settings", "", "json");
                m_Connection.LoadLayout(m_LayoutPath);
            }
            GUILayout.EndHorizontal();
        }

        void IHudTab.OnSelected() { }
        void IHudTab.OnDeselected() { }
    }
}
