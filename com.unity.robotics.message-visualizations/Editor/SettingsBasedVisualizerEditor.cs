using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;

public abstract class SettingsBasedVisualizerEditor<TMessageType, TVisualizerSettings> : Editor
    where TMessageType : Message
    where TVisualizerSettings : BaseVisualizerSettings<TMessageType>
{
    protected TVisualizerSettings m_Config;
    Editor m_Editor;

    public override void OnInspectorGUI()
    {
        DrawingVisualizerWithSettings<TMessageType, TVisualizerSettings> visualizer = (DrawingVisualizerWithSettings<TMessageType, TVisualizerSettings>)target;
        visualizer.Topic = EditorGUILayout.TextField("Topic", visualizer.Topic);

        m_Config = visualizer.Settings;
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

    public void VisualizerRedraw()
    {
        DrawingVisualizerWithSettings<TMessageType, TVisualizerSettings> visualizer = (DrawingVisualizerWithSettings<TMessageType, TVisualizerSettings>)target;
        visualizer.Redraw();
    }

    protected virtual void OnInspectorGUIForSettings(DrawingVisualizerWithSettings<TMessageType, TVisualizerSettings> visualizer)
    {
        CreateCachedEditor(m_Config, null, ref m_Editor);
        m_Editor.OnInspectorGUI();
    }
}
