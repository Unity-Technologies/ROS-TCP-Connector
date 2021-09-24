using RosMessageTypes.Sensor;
using System;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(MultiEchoLaserScanDefaultVisualizer))]
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

        if (m_Config.UseIntensitySize)
        {
            CreateMinMaxEditor("Size channel min", "Max", m_Config.SizeRange);
        }
        else
        {
            m_Config.PointRadius = float.Parse(EditorGUILayout.TextField("Point Radius", m_Config.PointRadius.ToString()));
        }
    }
}
