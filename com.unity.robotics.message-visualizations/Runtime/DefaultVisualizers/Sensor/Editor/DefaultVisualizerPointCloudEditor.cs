using System;
using UnityEditor;
using UnityEngine;

#if UNITY_EDITOR

[CustomEditor(typeof(DefaultVisualizerPointCloud))]
public class PointCloudEditor : Editor
{
    string m_ColorMax = "1000";
    float m_ColorMaxVal = 1000;
    string m_ColorMin = "0";
    float m_ColorMinVal;
    PointCloudVisualizerSettings m_Config;
    string m_SizeMax = "1000";
    float m_SizeMaxVal = 1000;
    string m_SizeMin = "0";
    float m_SizeMinVal;

    void Awake()
    {
        if (m_Config == null)
        {
            ((DefaultVisualizerPointCloud)target).settings = (PointCloudVisualizerSettings)AssetDatabase.LoadAssetAtPath("Packages/com.unity.robotics.message-visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects/PointCloudVisualizerSettings.asset", typeof(PointCloudVisualizerSettings));
            m_Config = ((DefaultVisualizerPointCloud)target).settings;
        }
    }

    void CreateNewDropdown(string label, string channel, Action<string> action)
    {
        if (m_Config.channels == null) return;

        GUILayout.BeginHorizontal();
        GUILayout.Label(label);
        if (EditorGUILayout.DropdownButton(new GUIContent(channel), FocusType.Keyboard))
        {
            var menu = new GenericMenu();
            foreach (var c in m_Config.channels)
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
        m_Config = (PointCloudVisualizerSettings)EditorGUILayout.ObjectField("Visualizer settings", m_Config, typeof(PointCloudVisualizerSettings), false);
        if (m_Config != null)
        {
            m_Config.topic = EditorGUILayout.TextField("Topic", m_Config.topic);
            m_Config.useSizeChannel = EditorGUILayout.ToggleLeft("Use size channel?", m_Config.useSizeChannel);

            if (m_Config.useSizeChannel)
            {
                MinMaxText("size", ref m_SizeMinVal, ref m_SizeMin, ref m_SizeMaxVal, ref m_SizeMax);
                CreateNewDropdown("Size channel name:", m_Config.sizeChannel, newChannel => { m_Config.sizeChannel = newChannel; });
                CreateMinMaxSlider(ref m_Config.sizeRange, m_SizeMinVal, m_SizeMaxVal);
            }

            m_Config.useRgbChannel = EditorGUILayout.ToggleLeft("Use color channel?", m_Config.useRgbChannel);

            if (m_Config.useRgbChannel)
            {
                m_Config.colorMode = (ColorMode)EditorGUILayout.EnumPopup("Color mode", m_Config.colorMode);

                MinMaxText("color", ref m_ColorMinVal, ref m_ColorMin, ref m_ColorMaxVal, ref m_ColorMax);

                switch (m_Config.colorMode)
                {
                    case ColorMode.HSV:
                        CreateNewDropdown("RGB channel name:", m_Config.rgbChannel, newChannel => { m_Config.rgbChannel = newChannel; });
                        CreateMinMaxSlider(ref m_Config.rgbRange, m_ColorMinVal, m_ColorMaxVal);
                        break;
                    case ColorMode.RGB:
                        m_Config.useSeparateRgb = EditorGUILayout.ToggleLeft("Separate R, G, B channels?", m_Config.useSeparateRgb);

                        if (m_Config.useSeparateRgb)
                        {
                            CreateNewDropdown("R channel name:", m_Config.rChannel, newChannel => { m_Config.rChannel = newChannel; });
                            CreateMinMaxSlider(ref m_Config.rRange, m_ColorMinVal, m_ColorMaxVal);

                            CreateNewDropdown("G channel name:", m_Config.gChannel, newChannel => { m_Config.gChannel = newChannel; });
                            CreateMinMaxSlider(ref m_Config.gRange, m_ColorMinVal, m_ColorMaxVal);

                            CreateNewDropdown("B channel name:", m_Config.bChannel, newChannel => { m_Config.bChannel = newChannel; });
                            CreateMinMaxSlider(ref m_Config.bRange, m_ColorMinVal, m_ColorMaxVal);
                        }
                        else
                        {
                            CreateNewDropdown("RGB channel name:", m_Config.rgbChannel, newChannel => { m_Config.rgbChannel = newChannel; });
                        }

                        break;
                }
            }
        }
    }
}

#endif //UNITY_EDITOR
