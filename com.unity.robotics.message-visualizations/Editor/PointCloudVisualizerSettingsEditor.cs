using System;
using RosMessageTypes.Sensor;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(DefaultVisualizerPointCloud))]
public class PointCloudVisualizerEditor : SettingsBasedVisualizerEditor<PointCloudMsg, PointCloudVisualizerSettings>
{
    string m_ColorMax = "1000";
    float m_ColorMaxVal = 1000;
    string m_ColorMin = "0";
    float m_ColorMinVal;
    string m_SizeMax = "1000";
    float m_SizeMaxVal = 1000;
    string m_SizeMin = "0";
    float m_SizeMinVal;

    void CreateNewDropdown(string label, PointCloudVisualizerSettings settings, string channel, Action<string> action)
    {
        if (settings.Channels == null)
            return;

        GUILayout.BeginHorizontal();
        GUILayout.Label(label);
        if (EditorGUILayout.DropdownButton(new GUIContent(channel), FocusType.Keyboard))
        {
            var menu = new GenericMenu();
            foreach (var c in settings.Channels)
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
        base.OnInspectorGUI();
        m_Config.UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel?", m_Config.UseSizeChannel);

        if (m_Config.UseSizeChannel)
        {
            MinMaxText("size", ref m_SizeMinVal, ref m_SizeMin, ref m_SizeMaxVal, ref m_SizeMax);
            CreateNewDropdown("Size channel name:", m_Config, m_Config.SizeChannel,
                newChannel => { m_Config.SizeChannel = newChannel; });
            var configSizeRange = m_Config.SizeRange;
            CreateMinMaxSlider(ref configSizeRange, m_SizeMinVal, m_SizeMaxVal);
            m_Config.SizeRange = configSizeRange;
        }

        m_Config.UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel?", m_Config.UseRgbChannel);

        if (m_Config.UseRgbChannel)
        {
            m_Config.ColorMode =
                (PointCloud2VisualizerSettings.ColorMode)EditorGUILayout.EnumPopup("Color mode", m_Config.ColorMode);

            MinMaxText("color", ref m_ColorMinVal, ref m_ColorMin, ref m_ColorMaxVal, ref m_ColorMax);

            switch (m_Config.ColorMode)
            {
                case PointCloud2VisualizerSettings.ColorMode.HSV:
                    CreateNewDropdown("RGB channel name:", m_Config, m_Config.HueChannel,
                        newChannel => { m_Config.HueChannel = newChannel; });
                    var configHueRange = m_Config.HueRange;
                    CreateMinMaxSlider(ref configHueRange, m_ColorMinVal, m_ColorMaxVal);
                    m_Config.HueRange = configHueRange;
                    break;
                case PointCloud2VisualizerSettings.ColorMode.RGB:
                    m_Config.UseSeparateRgb =
                        EditorGUILayout.ToggleLeft("Separate R, G, B channels?", m_Config.UseSeparateRgb);

                    if (m_Config.UseSeparateRgb)
                    {
                        CreateNewDropdown("R channel name:", m_Config, m_Config.RChannel,
                            newChannel => { m_Config.RChannel = newChannel; });
                        var configRRange = m_Config.RRange;
                        CreateMinMaxSlider(ref configRRange, m_ColorMinVal, m_ColorMaxVal);
                        m_Config.RRange = configRRange;

                        CreateNewDropdown("G channel name:", m_Config, m_Config.GChannel,
                            newChannel => { m_Config.GChannel = newChannel; });
                        var configGRange = m_Config.GRange;
                        CreateMinMaxSlider(ref configGRange, m_ColorMinVal, m_ColorMaxVal);
                        m_Config.GRange = configGRange;

                        CreateNewDropdown("B channel name:", m_Config, m_Config.BChannel,
                            newChannel => { m_Config.BChannel = newChannel; });
                        var configBRange = m_Config.BRange;
                        CreateMinMaxSlider(ref configBRange, m_ColorMinVal, m_ColorMaxVal);
                        m_Config.BRange = configBRange;
                    }
                    else
                    {
                        CreateNewDropdown("RGB channel name:", m_Config, m_Config.HueChannel,
                            newChannel => { m_Config.HueChannel = newChannel; });
                    }

                    break;
            }
        }
    }
}
