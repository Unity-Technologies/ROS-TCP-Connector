using System;
using UnityEditor;
using UnityEngine;

#if UNITY_EDITOR

[CustomEditor(typeof(DefaultVisualizerLaserScan))]
public class LaserScanEditor : Editor
{
    LaserScanVisualizerSettings m_Config;
    string m_SizeMax = "1000";
    float m_SizeMaxVal = 1000;
    string m_SizeMin = "0";
    float m_SizeMinVal;

    void Awake()
    {
        if (m_Config == null)
        {
            ((DefaultVisualizerLaserScan)target).settings = (LaserScanVisualizerSettings)AssetDatabase.LoadAssetAtPath("Packages/com.unity.robotics.message-visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects/LaserScanVisualizerSettings.asset", typeof(LaserScanVisualizerSettings));
            m_Config = ((DefaultVisualizerLaserScan)target).settings;
        }
    }

    void CreateMinMaxSlider(ref float[] range, float min, float max)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(range[0].ToString());
        EditorGUILayout.MinMaxSlider(ref range[0], ref range[1], min, max);
        GUILayout.Label(range[1].ToString());
        GUILayout.EndHorizontal();
    }

    void MinMaxText(string label, ref float minVal, ref string minS, ref float maxVal, ref string maxS)
    {
        minVal = float.Parse(EditorGUILayout.TextField($"Min {label} range:", minS));
        minS = minVal.ToString();

        maxVal = float.Parse(EditorGUILayout.TextField($"Max {label} range:", maxS));
        maxS = maxVal.ToString();
    }

    public override void OnInspectorGUI()
    {
        m_Config = (LaserScanVisualizerSettings)EditorGUILayout.ObjectField("Visualizer settings", m_Config, typeof(LaserScanVisualizerSettings), false);
        if (m_Config != null)
        {
            m_Config.topic = EditorGUILayout.TextField("Topic", m_Config.topic);
            m_Config.useIntensitySize = EditorGUILayout.ToggleLeft("Use intensity size?", m_Config.useIntensitySize);

            if (m_Config.useIntensitySize)
            {
                MinMaxText("size", ref m_SizeMinVal, ref m_SizeMin, ref m_SizeMaxVal, ref m_SizeMax);
                CreateMinMaxSlider(ref m_Config.sizeRange, m_SizeMinVal, m_SizeMaxVal);
            }
        }
    }
}

#endif //UNITY_EDITOR
