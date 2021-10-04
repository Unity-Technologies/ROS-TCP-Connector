using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.Visualizations;
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

    public void CreateNewDropdown(string[] channels, string label, string channel, System.Action<string> action)
    {
        if (channels == null)
            return;

        GUILayout.BeginHorizontal();
        GUILayout.Label(label);
        if (EditorGUILayout.DropdownButton(new GUIContent(channel), FocusType.Keyboard))
        {
            var menu = new GenericMenu();
            foreach (var c in channels)
                menu.AddItem(new GUIContent(c), c == channel, () =>
                {
                    action(c);
                    VisualizerRedraw();
                });
            menu.DropDown(new Rect(Event.current.mousePosition.x, Event.current.mousePosition.y, 0f, 0f));
        }

        GUILayout.EndHorizontal();
    }

    // NB, this modifies the 'range' array in place
    public void CreateMinMaxEditor(string label1, string label2, float[] range)
    {
        GUILayout.BeginHorizontal();
        range[0] = EditorGUILayout.FloatField(label1, range[0]);
        //GUILayout.Space(30);
        GUILayout.Label(label2);
        range[1] = EditorGUILayout.FloatField(range[1], GUILayout.Width(70));
        GUILayout.EndHorizontal();
    }

}
