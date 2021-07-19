using RosMessageTypes.Sensor;
using System;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(DefaultVisualizerMultiEchoLaserScan))]
public class MultiEchoLaserScanEditor : SettingsBasedVisualizerEditor<MultiEchoLaserScanMsg, MultiEchoLaserScanVisualizerSettings>
{
    string m_SizeMax = "1000";
    float m_SizeMaxVal = 1000;
    string m_SizeMin = "0";
    float m_SizeMinVal;

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
        base.OnInspectorGUI();
        m_Config.UseIntensitySize = EditorGUILayout.ToggleLeft("Use intensity size?", m_Config.UseIntensitySize);

        if (m_Config.UseIntensitySize)
        {
            MinMaxText("size", ref m_SizeMinVal, ref m_SizeMin, ref m_SizeMaxVal, ref m_SizeMax);
            var configSizeRange = m_Config.SizeRange;
            CreateMinMaxSlider(ref configSizeRange, m_SizeMinVal, m_SizeMaxVal);
            m_Config.SizeRange = configSizeRange;
        }
    }
}
