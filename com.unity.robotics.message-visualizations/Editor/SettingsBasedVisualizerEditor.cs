using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;

public abstract class SettingsBasedVisualizerEditor<TMessageType, TVisualizerSettings> : Editor
    where TMessageType : Message
    where TVisualizerSettings : VisualizerSettings<TMessageType>
{
    public const string ScriptableObjectsSettingsPath = "Assets/ROS-TCP-Connector/com.unity.robotics.message-visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects/";
    public abstract string DefaultScriptableObjectPath { get; }

    TVisualizerSettings m_Config;
    Editor m_Editor;

    bool m_LoggedNullConfigWarning;

    public override void OnInspectorGUI()
    {
        var visualizer = (SettingsBasedVisualFactory<TMessageType, TVisualizerSettings>)target;
        visualizer.Topic = EditorGUILayout.TextField("Topic", visualizer.Topic);

        m_Config ??= visualizer.Settings ??=
            AssetDatabase.LoadAssetAtPath<TVisualizerSettings>(DefaultScriptableObjectPath);
        if (m_Config == null)
        {
            if (!m_LoggedNullConfigWarning)
            {
                Debug.LogWarning(
                    $"Failed to find default settings for {GetType()} -- you will have to find them manually.");
                m_LoggedNullConfigWarning = true;
            }
        }
        else
        {
            m_LoggedNullConfigWarning = false;
        }

        GUI.changed = false;
        m_Config = (TVisualizerSettings)EditorGUILayout.ObjectField(
            "Visualizer settings", m_Config, typeof(TVisualizerSettings), false);

        if (m_Config != null)
        {
            visualizer.Settings = m_Config;
            EditorGUI.indentLevel++;
            OnInspectorGUIForSettings(visualizer);
            EditorGUI.indentLevel--;
        }


        if (GUI.changed)
        {
            visualizer.Redraw();
        }
    }

    protected virtual void OnInspectorGUIForSettings(SettingsBasedVisualFactory<TMessageType, TVisualizerSettings> visualizer)
    {
        if (m_Editor == null)
        {
            CreateCachedEditor(m_Config, null, ref m_Editor);
        }
        m_Editor.OnInspectorGUI();
    }
}
