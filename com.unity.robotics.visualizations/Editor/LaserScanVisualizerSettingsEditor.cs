using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.Visualizations;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(LaserScanDefaultVisualizer))]
public class LaserScanEditor : SettingsBasedVisualizerEditor<LaserScanMsg, LaserScanVisualizerSettings>
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        if (m_Config.UseIntensitySize)
        {
            m_Config.MaxIntensity = float.Parse(EditorGUILayout.TextField("Max Intensity", m_Config.MaxIntensity.ToString()));
        }
        else
        {
            m_Config.PointRadius = float.Parse(EditorGUILayout.TextField("Point Radius", m_Config.PointRadius.ToString()));
        }
    }
}
