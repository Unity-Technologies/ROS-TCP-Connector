using System;
using RosMessageTypes.Sensor;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(PointCloudDefaultVisualizer))]
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

    // NB, modifies 'range' array in place
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
        m_Config.UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel", m_Config.UseSizeChannel);

        if (m_Config.UseSizeChannel)
        {
            MinMaxText("size", ref m_SizeMinVal, ref m_SizeMin, ref m_SizeMaxVal, ref m_SizeMax);
            CreateNewDropdown("Size channel name:", m_Config, m_Config.SizeChannel,
                newChannel => { m_Config.SizeChannel = newChannel; });
            CreateMinMaxSlider(m_Config.SizeRange, m_SizeMinVal, m_SizeMaxVal);
        }

        m_Config.UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel", m_Config.UseRgbChannel);

        if (m_Config.UseRgbChannel)
        {
            m_Config.ColorMode =
                (PointCloud2VisualizerSettings.ColorMode)EditorGUILayout.EnumPopup("Color mode", m_Config.ColorMode);

            MinMaxText("color", ref m_ColorMinVal, ref m_ColorMin, ref m_ColorMaxVal, ref m_ColorMax);

            switch (m_Config.ColorMode)
            {
                case PointCloud2VisualizerSettings.ColorMode.HSV:
                    CreateNewDropdown("Hue channel name:", m_Config, m_Config.HueChannel, newChannel => { m_Config.HueChannel = newChannel; });
                    CreateMinMaxSlider(m_Config.HueRange, m_ColorMinVal, m_ColorMaxVal);
                    break;
                case PointCloud2VisualizerSettings.ColorMode.CombinedRGB:
                    CreateNewDropdown("RGB channel name:", m_Config, m_Config.RgbChannel, newChannel => { m_Config.RgbChannel = newChannel; });
                    break;
                case PointCloud2VisualizerSettings.ColorMode.SeparateRGB:
                    CreateNewDropdown("R channel name:", m_Config, m_Config.RChannel, newChannel => { m_Config.RChannel = newChannel; });
                    CreateMinMaxSlider(m_Config.RRange, m_ColorMinVal, m_ColorMaxVal);

                    CreateNewDropdown("G channel name:", m_Config, m_Config.GChannel, newChannel => { m_Config.GChannel = newChannel; });
                    CreateMinMaxSlider(m_Config.GRange, m_ColorMinVal, m_ColorMaxVal);

                    CreateNewDropdown("B channel name:", m_Config, m_Config.BChannel, newChannel => { m_Config.BChannel = newChannel; });
                    CreateMinMaxSlider(m_Config.BRange, m_ColorMinVal, m_ColorMaxVal);
                    break;
            }
        }
    }
}
