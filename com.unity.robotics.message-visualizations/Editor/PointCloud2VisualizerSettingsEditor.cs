using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.MessageVisualizers;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(PointCloud2DefaultVisualizer))]
public class PointCloud2Editor : SettingsBasedVisualizerEditor<PointCloud2Msg, PointCloud2VisualizerSettings>
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
        base.OnInspectorGUI();

        CreateNewDropdown(m_Config.Channels, "X channel name:", m_Config.XChannel, newChannel => { m_Config.XChannel = newChannel; });
        CreateNewDropdown(m_Config.Channels, "Y channel name:", m_Config.YChannel, newChannel => { m_Config.YChannel = newChannel; });
        CreateNewDropdown(m_Config.Channels, "Z channel name:", m_Config.ZChannel, newChannel => { m_Config.ZChannel = newChannel; });

        m_Config.UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel?", m_Config.UseSizeChannel);

        if (m_Config.UseSizeChannel)
        {
            MinMaxText("size", ref sizeMinVal, ref sizeMin, ref sizeMaxVal, ref sizeMax);
            CreateNewDropdown(m_Config.Channels, "Size channel name:", m_Config.SizeChannel, newChannel => { m_Config.SizeChannel = newChannel; });
            var configSizeRange = m_Config.SizeRange;
            CreateMinMaxSlider(ref configSizeRange, sizeMinVal, sizeMaxVal);
            m_Config.SizeRange = configSizeRange;
        }

        m_Config.UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel?", m_Config.UseRgbChannel);

        if (m_Config.UseRgbChannel)
        {
            m_Config.ColorModeSetting = (PointCloud2VisualizerSettings.ColorMode)EditorGUILayout.EnumPopup("Color mode", m_Config.ColorModeSetting);

            if (settings.colorMode != PointCloud2VisualizerSettings.ColorMode.CombinedRGB)
                MinMaxText("color", ref colorMinVal, ref colorMin, ref colorMaxVal, ref colorMax);

            switch (m_Config.ColorModeSetting)
            {
                case PointCloud2VisualizerSettings.ColorMode.HSV:
                    CreateNewDropdown(m_Config.Channels, "Hue channel name:", m_Config.HueChannel, newChannel => { m_Config.HueChannel = newChannel; });
                    var configHueRange = m_Config.HueRange;
                    CreateMinMaxSlider(ref configHueRange, colorMinVal, colorMaxVal);
                    m_Config.HueRange = configHueRange;
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
