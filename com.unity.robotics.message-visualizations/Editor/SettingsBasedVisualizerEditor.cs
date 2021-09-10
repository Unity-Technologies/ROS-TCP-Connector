using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;

public abstract class SettingsBasedVisualizerEditor<TMessageType, TVisualizerSettings> : Editor
    where TMessageType : Message
    where TVisualizerSettings : VisualizerSettingsGeneric<TMessageType>
{
    protected TVisualizerSettings m_Config;
    Editor m_Editor;

    public override void OnInspectorGUI()
    {
        DrawingVisualizerWithSettings<TMessageType, TVisualizerSettings> visualizer = (DrawingVisualizerWithSettings<TMessageType, TVisualizerSettings>)target;
        visualizer.Topic = EditorGUILayout.TextField("Topic", visualizer.Topic);

        m_Config = visualizer.Settings;
        m_Config = (TVisualizerSettings)EditorGUILayout.ObjectField("Visualizer settings", m_Config, typeof(TVisualizerSettings), false);
        visualizer.Settings = m_Config;

        if (GUI.changed)
        {
            EditorUtility.SetDirty(target);
            if (visualizer.gameObject.scene != null)
            {
                UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(visualizer.gameObject.scene);
            }
            GUI.changed = false;
        }

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
