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
                    VisualizerRedraw();
                });
            menu.DropDown(new Rect(Event.current.mousePosition.x, Event.current.mousePosition.y, 0f, 0f));
        }

        GUILayout.EndHorizontal();
    }

    // NB, this modifies the 'range' array in place
    void CreateMinMaxSlider(float[] range, float min, float max)
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

        GUI.changed = false;
        CreateNewDropdown(m_Config.Channels, "X channel name:", m_Config.XChannel, newChannel => { m_Config.XChannel = newChannel; });
        CreateNewDropdown(m_Config.Channels, "Y channel name:", m_Config.YChannel, newChannel => { m_Config.YChannel = newChannel; });
        CreateNewDropdown(m_Config.Channels, "Z channel name:", m_Config.ZChannel, newChannel => { m_Config.ZChannel = newChannel; });

        m_Config.UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel?", m_Config.UseSizeChannel);

        if (m_Config.UseSizeChannel)
        {
            MinMaxText("size", ref sizeMinVal, ref sizeMin, ref sizeMaxVal, ref sizeMax);
            CreateNewDropdown(m_Config.Channels, "Size channel name:", m_Config.SizeChannel, newChannel => { m_Config.SizeChannel = newChannel; });
            CreateMinMaxSlider(m_Config.SizeRange, sizeMinVal, sizeMaxVal);
        }
        else
        {
            m_Config.Size = EditorGUILayout.FloatField("Size:", m_Config.Size);
        }

        m_Config.UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel?", m_Config.UseRgbChannel);

        if (m_Config.UseRgbChannel)
        {
            m_Config.ColorModeSetting = (PointCloud2VisualizerSettings.ColorMode)EditorGUILayout.EnumPopup("Color mode", m_Config.ColorModeSetting);

            if (m_Config.ColorModeSetting != PointCloud2VisualizerSettings.ColorMode.CombinedRGB)
                MinMaxText("color", ref colorMinVal, ref colorMin, ref colorMaxVal, ref colorMax);

            switch (m_Config.ColorModeSetting)
            {
                case PointCloud2VisualizerSettings.ColorMode.HSV:
                    CreateNewDropdown(m_Config.Channels, "Hue channel name:", m_Config.HueChannel, newChannel => { m_Config.HueChannel = newChannel; });
                    CreateMinMaxSlider(m_Config.HueRange, colorMinVal, colorMaxVal);
                    break;
                case PointCloud2VisualizerSettings.ColorMode.CombinedRGB:
                    CreateNewDropdown(m_Config.Channels, "Rgb channel name:", m_Config.RgbChannel, newChannel => { m_Config.RgbChannel = newChannel; });
                    break;
                case PointCloud2VisualizerSettings.ColorMode.SeparateRGB:
                    CreateNewDropdown(m_Config.Channels, "R channel name:", m_Config.RChannel, newChannel => { m_Config.RChannel = newChannel; });
                    CreateMinMaxSlider(m_Config.RRange, colorMinVal, colorMaxVal);

                    CreateNewDropdown(m_Config.Channels, "G channel name:", m_Config.GChannel, newChannel => { m_Config.GChannel = newChannel; });
                    CreateMinMaxSlider(m_Config.GRange, colorMinVal, colorMaxVal);

                    CreateNewDropdown(m_Config.Channels, "B channel name:", m_Config.BChannel, newChannel => { m_Config.BChannel = newChannel; });
                    CreateMinMaxSlider(m_Config.BRange, colorMinVal, colorMaxVal);
                    break;
            }
        }

        if (GUI.changed)
        {
            VisualizerRedraw();
        }
    }
}
