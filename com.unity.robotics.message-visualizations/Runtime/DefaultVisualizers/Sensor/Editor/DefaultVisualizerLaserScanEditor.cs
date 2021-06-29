using System;
using UnityEditor;
using UnityEngine;

#if UNITY_EDITOR

[CustomEditor(typeof(DefaultVisualizerLaserScan))]
public class LaserScanEditor : Editor
{
    LaserScanVisualizerSettings config;
    string sizeMax = "1000";
    float sizeMaxVal = 1000;
    string sizeMin = "0";
    float sizeMinVal;

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
        config = (LaserScanVisualizerSettings)EditorGUILayout.ObjectField("Visualizer settings", config, typeof(LaserScanVisualizerSettings), false);
        if (config == null) config = (LaserScanVisualizerSettings)AssetDatabase.LoadAssetAtPath("Packages/com.unity.robotics.message-visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects/LaserScanVisualizerSettings.asset", typeof(LaserScanVisualizerSettings));
        ((DefaultVisualizerLaserScan)target).m_Settings = config;

        config.m_UseIntensitySize = EditorGUILayout.ToggleLeft("Use intensity size?", config.m_UseIntensitySize);

        if (config.m_UseIntensitySize)
        {
            MinMaxText("size", ref sizeMinVal, ref sizeMin, ref sizeMaxVal, ref sizeMax);
            CreateMinMaxSlider(ref config.m_SizeRange, sizeMinVal, sizeMaxVal);
        }
    }
}

#endif //UNITY_EDITOR
