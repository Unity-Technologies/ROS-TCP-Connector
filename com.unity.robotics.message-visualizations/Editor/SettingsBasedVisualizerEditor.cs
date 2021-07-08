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
    public const string ScriptableObjectsSettingsPath = "Packages/com.unity.robotics.message-visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects";
    public abstract string DefaultScriptableObjectFilename { get; }

    TVisualizerSettings m_Config;
    Editor m_Editor;

    public override void OnInspectorGUI()
    {
        SettingsBasedVisualFactory<TMessageType, TVisualizerSettings> visualizer = (SettingsBasedVisualFactory<TMessageType, TVisualizerSettings>)target;
        visualizer.Topic = EditorGUILayout.TextField("Topic", visualizer.Topic);

        if (m_Config == null)
        {
            m_Config = visualizer.Settings;
            if (m_Config == null)
                m_Config = (TVisualizerSettings)AssetDatabase.LoadAssetAtPath(System.IO.Path.Combine(ScriptableObjectsSettingsPath, DefaultScriptableObjectFilename), typeof(TVisualizerSettings));
        }
        GUI.changed = false;
        m_Config = (TVisualizerSettings)EditorGUILayout.ObjectField("Visualizer settings", m_Config, typeof(TVisualizerSettings), false);
        visualizer.Settings = m_Config;

        EditorGUI.indentLevel++;
        OnInspectorGUIForSettings(visualizer);
        EditorGUI.indentLevel--;

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
