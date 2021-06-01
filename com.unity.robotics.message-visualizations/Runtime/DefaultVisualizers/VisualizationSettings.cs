using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEditor;
using IHudTab = Unity.Robotics.ROSTCPConnector.IHudTab;

namespace Unity.Robotics.MessageVisualizers
{
    public class VisualizationSettings : MonoBehaviour, ROSTCPConnector.IHudTab
    {
        string IHudTab.Label => "Settings";
        string m_LayoutPath;

        IEnumerator Start()
        {
            // Add settings to HUD last
            yield return new WaitForEndOfFrame();
            HUDPanel.RegisterTab(this);
        }

        void IHudTab.OnGUI(HUDPanel hud) 
        { 
            // Save/Load layout files
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Save layout"))
            {
                m_LayoutPath = EditorUtility.SaveFilePanel("Save Visualizations Settings", "", "RosHudLayout", "json");
                hud.SaveLayout(m_LayoutPath);
            }
            if (GUILayout.Button("Load layout"))
            {
                m_LayoutPath = EditorUtility.OpenFilePanel("Select Visualizations Settings", "", "json");
                hud.LoadLayout(m_LayoutPath);
            }
            GUILayout.EndHorizontal();
        }
        void IHudTab.OnSelected() { }
        void IHudTab.OnDeselected() { }
    }
}