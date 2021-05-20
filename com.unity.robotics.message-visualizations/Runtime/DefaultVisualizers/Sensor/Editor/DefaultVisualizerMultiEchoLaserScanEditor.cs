using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using UnityEditor;

#if UNITY_EDITOR

[CustomEditor(typeof(DefaultVisualizerMultiEchoLaserScan))]
public class MultiEchoLaserScanEditor : Editor
{
    MultiEchoLaserScanVisualizerSettings config;
    string sizeMin = "0";
    string sizeMax = "1000";
    float sizeMinVal = 0;
    float sizeMaxVal = 1000;

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
        config = (MultiEchoLaserScanVisualizerSettings)EditorGUILayout.ObjectField("Visualizer settings", config, typeof(MultiEchoLaserScanVisualizerSettings), false);
        if (config == null)
        {
            config = (MultiEchoLaserScanVisualizerSettings)AssetDatabase.LoadAssetAtPath("Packages/com.unity.robotics.message-visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects/MultiEchoLaserScanVisualizerSettings.asset", typeof(MultiEchoLaserScanVisualizerSettings));
        }
        ((DefaultVisualizerMultiEchoLaserScan)target).m_Settings = config;

        config.m_UseIntensitySize = EditorGUILayout.ToggleLeft("Use intensity size?", config.m_UseIntensitySize);

        if (config.m_UseIntensitySize)
        {
            MinMaxText("size", ref sizeMinVal, ref sizeMin, ref sizeMaxVal, ref sizeMax);
            CreateMinMaxSlider(ref config.m_SizeRange, sizeMinVal, sizeMaxVal);
        }
   }
}

#endif //UNITY_EDITOR