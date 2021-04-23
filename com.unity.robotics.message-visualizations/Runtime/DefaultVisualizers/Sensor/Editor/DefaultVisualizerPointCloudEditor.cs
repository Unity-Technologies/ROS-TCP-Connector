using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(DefaultVisualizerPointCloud))]
public class PointCloudEditor : Editor
{
    PointCloudVisualizerSettings pclConfig;
    string colorMin = "0";
    string colorMax = "1000";
    float colorMinVal = 0;
    float colorMaxVal = 1000;
    string sizeMin = "0";
    string sizeMax = "1000";
    float sizeMinVal = 0;
    float sizeMaxVal = 1000;

    void CreateNewDropdown(string label, string channel, Action<string> action)
    {
        if (pclConfig.channels == null)
        {
            return;
        }

        GUILayout.BeginHorizontal();
        GUILayout.Label(label);
        if (EditorGUILayout.DropdownButton(new GUIContent(channel), FocusType.Keyboard))
        {
            GenericMenu menu = new GenericMenu();
            foreach (var c in pclConfig.channels)
            {
                menu.AddItem(new GUIContent(c.name), c.name == channel, () => { 
                    action(c.name);
                });
            }
            menu.DropDown(new Rect(Event.current.mousePosition.x, Event.current.mousePosition.y, 0f, 0f));
        }
        GUILayout.EndHorizontal();
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
        pclConfig = (PointCloudVisualizerSettings)EditorGUILayout.ObjectField("Visualizer settings", pclConfig, typeof(PointCloudVisualizerSettings), false);
        if (pclConfig == null)
        {
            pclConfig = (PointCloudVisualizerSettings)AssetDatabase.LoadAssetAtPath("Packages/com.unity.robotics.message-visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects/PointCloudVisualizerSettings.asset", typeof(PointCloudVisualizerSettings));
        }
        ((DefaultVisualizerPointCloud)target).m_Settings = pclConfig;

        pclConfig.m_UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel?", pclConfig.m_UseSizeChannel);

        if (pclConfig.m_UseSizeChannel)
        {
            MinMaxText("size", ref sizeMinVal, ref sizeMin, ref sizeMaxVal, ref sizeMax);
            CreateNewDropdown("Size channel name:", pclConfig.m_SizeChannel, (newChannel) => { pclConfig.m_SizeChannel = newChannel; });
            CreateMinMaxSlider(ref pclConfig.m_SizeRange, sizeMinVal, sizeMaxVal);
        }

        pclConfig.m_UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel?", pclConfig.m_UseRgbChannel);

        if (pclConfig.m_UseRgbChannel)
        {
            pclConfig.colorMode = (ColorMode)EditorGUILayout.EnumPopup("Color mode", pclConfig.colorMode);

            MinMaxText("color", ref colorMinVal, ref colorMin, ref colorMaxVal, ref colorMax);

            switch (pclConfig.colorMode)
            {
                case ColorMode.HSV:
                    CreateNewDropdown("RGB channel name:", pclConfig.m_RgbChannel, (newChannel) => { pclConfig.m_RgbChannel = newChannel; });
                    CreateMinMaxSlider(ref pclConfig.m_RgbRange, colorMinVal, colorMaxVal);
                    break;
                case ColorMode.RGB:
                    CreateNewDropdown("R channel name:", pclConfig.m_RChannel, (newChannel) => { pclConfig.m_RChannel = newChannel; });
                    CreateMinMaxSlider(ref pclConfig.m_RRange, colorMinVal, colorMaxVal);

                    CreateNewDropdown("G channel name:", pclConfig.m_GChannel, (newChannel) => { pclConfig.m_GChannel = newChannel; });
                    CreateMinMaxSlider(ref pclConfig.m_GRange, colorMinVal, colorMaxVal);

                    CreateNewDropdown("B channel name:", pclConfig.m_BChannel, (newChannel) => { pclConfig.m_BChannel = newChannel; });
                    CreateMinMaxSlider(ref pclConfig.m_BRange, colorMinVal, colorMaxVal);
                    break;
            }
        }
   }
}