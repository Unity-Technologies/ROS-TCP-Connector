using System;
using UnityEditor;
using UnityEngine;

#if UNITY_EDITOR

[CustomEditor(typeof(DefaultVisualizerPointCloud2))]
public class PointCloud2Editor : Editor
{
    string colorMax = "1000";
    float colorMaxVal = 1000;
    string colorMin = "0";
    float colorMinVal;
    PointCloud2VisualizerSettings pcl2Config;
    string sizeMax = "1000";
    float sizeMaxVal = 1000;
    string sizeMin = "0";
    float sizeMinVal;

    void Awake()
    {
        if (pcl2Config == null)
        {
            ((DefaultVisualizerPointCloud2)target).m_Settings = (PointCloud2VisualizerSettings)AssetDatabase.LoadAssetAtPath("Packages/com.unity.robotics.message-visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects/PointCloud2VisualizerSettings.asset", typeof(PointCloud2VisualizerSettings));
            pcl2Config = ((DefaultVisualizerPointCloud2)target).m_Settings;
        }
    }

    void CreateNewDropdown(string label, string channel, Action<string> action)
    {
        if (pcl2Config.channels == null) return;

        GUILayout.BeginHorizontal();
        GUILayout.Label(label);
        if (EditorGUILayout.DropdownButton(new GUIContent(channel), FocusType.Keyboard))
        {
            var menu = new GenericMenu();
            foreach (var c in pcl2Config.channels)
                menu.AddItem(new GUIContent(c.name), c.name == channel, () =>
                {
                    action(c.name);
                });
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
        pcl2Config = (PointCloud2VisualizerSettings)EditorGUILayout.ObjectField("Visualizer settings", pcl2Config, typeof(PointCloud2VisualizerSettings), false);

        if (pcl2Config != null)
        {
            CreateNewDropdown("X channel name:", pcl2Config.m_XChannel, newChannel => { pcl2Config.m_XChannel = newChannel; });
            CreateNewDropdown("Y channel name:", pcl2Config.m_YChannel, newChannel => { pcl2Config.m_YChannel = newChannel; });
            CreateNewDropdown("Z channel name:", pcl2Config.m_ZChannel, newChannel => { pcl2Config.m_ZChannel = newChannel; });

            pcl2Config.m_UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel?", pcl2Config.m_UseSizeChannel);

            if (pcl2Config.m_UseSizeChannel)
            {
                MinMaxText("size", ref sizeMinVal, ref sizeMin, ref sizeMaxVal, ref sizeMax);
                CreateNewDropdown("Size channel name:", pcl2Config.m_SizeChannel, newChannel => { pcl2Config.m_SizeChannel = newChannel; });
                CreateMinMaxSlider(ref pcl2Config.m_SizeRange, sizeMinVal, sizeMaxVal);
            }

            pcl2Config.m_UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel?", pcl2Config.m_UseRgbChannel);

            if (pcl2Config.m_UseRgbChannel)
            {
                pcl2Config.colorMode = (ColorMode)EditorGUILayout.EnumPopup("Color mode", pcl2Config.colorMode);

                MinMaxText("color", ref colorMinVal, ref colorMin, ref colorMaxVal, ref colorMax);

                switch (pcl2Config.colorMode)
                {
                    case ColorMode.HSV:
                        CreateNewDropdown("RGB channel name:", pcl2Config.m_RgbChannel, newChannel => { pcl2Config.m_RgbChannel = newChannel; });
                        CreateMinMaxSlider(ref pcl2Config.m_RgbRange, colorMinVal, colorMaxVal);
                        break;
                    case ColorMode.RGB:
                        CreateNewDropdown("R channel name:", pcl2Config.m_RChannel, newChannel => { pcl2Config.m_RChannel = newChannel; });
                        CreateMinMaxSlider(ref pcl2Config.m_RRange, colorMinVal, colorMaxVal);

                        CreateNewDropdown("G channel name:", pcl2Config.m_GChannel, newChannel => { pcl2Config.m_GChannel = newChannel; });
                        CreateMinMaxSlider(ref pcl2Config.m_GRange, colorMinVal, colorMaxVal);

                        CreateNewDropdown("B channel name:", pcl2Config.m_BChannel, newChannel => { pcl2Config.m_BChannel = newChannel; });
                        CreateMinMaxSlider(ref pcl2Config.m_BRange, colorMinVal, colorMaxVal);
                        break;
                }
            }
        }
    }
}

#endif //UNITY_EDITOR
