using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.MessageVisualizers;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(PointCloud2DefaultVisualizer))]
public class PointCloud2Editor : SettingsBasedVisualizerEditor<PointCloud2Msg, PointCloud2VisualizerSettings>
{
}

[CustomEditor(typeof(PointCloud2VisualizerSettings))]
public class PointCloud2VisualizerSettingsEditor : Editor
{
    string colorMax = "1000";
    float colorMaxVal = 1000;
    string colorMin = "0";
    float colorMinVal;
    string sizeMax = "1000";
    float sizeMaxVal = 1000;
    string sizeMin = "0";
    float sizeMinVal;

    void CreateNewDropdown(string[] channels, string label, string channel, Action<string> action)
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
        PointCloud2VisualizerSettings settings = (PointCloud2VisualizerSettings)target;

        CreateNewDropdown(settings.channels, "X channel name:", settings.m_XChannel, newChannel => { settings.m_XChannel = newChannel; });
        CreateNewDropdown(settings.channels, "Y channel name:", settings.m_YChannel, newChannel => { settings.m_YChannel = newChannel; });
        CreateNewDropdown(settings.channels, "Z channel name:", settings.m_ZChannel, newChannel => { settings.m_ZChannel = newChannel; });

        settings.m_UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel?", settings.m_UseSizeChannel);

        if (settings.m_UseSizeChannel)
        {
            MinMaxText("size", ref sizeMinVal, ref sizeMin, ref sizeMaxVal, ref sizeMax);
            CreateNewDropdown(settings.channels, "Size channel name:", settings.m_SizeChannel, newChannel => { settings.m_SizeChannel = newChannel; });
            CreateMinMaxSlider(ref settings.m_SizeRange, sizeMinVal, sizeMaxVal);
        }

        settings.m_UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel?", settings.m_UseRgbChannel);

        if (settings.m_UseRgbChannel)
        {
            settings.colorMode = (PointCloud2VisualizerSettings.ColorMode)EditorGUILayout.EnumPopup("Color mode", settings.colorMode);

            if (settings.colorMode != PointCloud2VisualizerSettings.ColorMode.CombinedRGB)
                MinMaxText("color", ref colorMinVal, ref colorMin, ref colorMaxVal, ref colorMax);

            switch (settings.colorMode)
            {
                case PointCloud2VisualizerSettings.ColorMode.HSV:
                    CreateNewDropdown(settings.channels, "Hue channel name:", settings.m_HueChannel, newChannel => { settings.m_HueChannel = newChannel; });
                    CreateMinMaxSlider(ref settings.m_HueRange, colorMinVal, colorMaxVal);
                    break;
                case PointCloud2VisualizerSettings.ColorMode.CombinedRGB:
                    CreateNewDropdown(settings.channels, "Rgb channel name:", settings.m_RgbChannel, newChannel => { settings.m_RgbChannel = newChannel; });
                    break;
                case PointCloud2VisualizerSettings.ColorMode.SeparateRGB:
                    CreateNewDropdown(settings.channels, "R channel name:", settings.m_RChannel, newChannel => { settings.m_RChannel = newChannel; });
                    CreateMinMaxSlider(ref settings.m_RRange, colorMinVal, colorMaxVal);

                    CreateNewDropdown(settings.channels, "G channel name:", settings.m_GChannel, newChannel => { settings.m_GChannel = newChannel; });
                    CreateMinMaxSlider(ref settings.m_GRange, colorMinVal, colorMaxVal);

                    CreateNewDropdown(settings.channels, "B channel name:", settings.m_BChannel, newChannel => { settings.m_BChannel = newChannel; });
                    CreateMinMaxSlider(ref settings.m_BRange, colorMinVal, colorMaxVal);
                    break;
            }
        }
    }
}
