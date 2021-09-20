using System;
using RosMessageTypes.Sensor;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(PointCloudDefaultVisualizer))]
public class PointCloudVisualizerEditor : SettingsBasedVisualizerEditor<PointCloudMsg, PointCloudVisualizerSettings>
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        m_Config.UseSizeChannel = EditorGUILayout.ToggleLeft("Use size channel", m_Config.UseSizeChannel);

        if (m_Config.UseSizeChannel)
        {
            m_Config.Size = EditorGUILayout.FloatField("Max Size:", m_Config.Size);
            CreateNewDropdown(m_Config.Channels, "Size channel name:", m_Config.SizeChannel,
                newChannel => { m_Config.SizeChannel = newChannel; });
            CreateMinMaxEditor("Size channel min", "Max", m_Config.SizeRange);
        }
        else
        {
            m_Config.Size = EditorGUILayout.FloatField("Size:", m_Config.Size);
        }

        m_Config.UseRgbChannel = EditorGUILayout.ToggleLeft("Use color channel", m_Config.UseRgbChannel);

        if (m_Config.UseRgbChannel)
        {
            m_Config.ColorMode =
                (PointCloud2VisualizerSettings.ColorMode)EditorGUILayout.EnumPopup("Color mode", m_Config.ColorMode);

            switch (m_Config.ColorMode)
            {
                case PointCloud2VisualizerSettings.ColorMode.HSV:
                    CreateNewDropdown(m_Config.Channels, "Hue channel name:", m_Config.HueChannel, newChannel => { m_Config.HueChannel = newChannel; });
                    CreateMinMaxEditor("Hue channel min", "Max", m_Config.HueRange);
                    break;
                case PointCloud2VisualizerSettings.ColorMode.CombinedRGB:
                    CreateNewDropdown(m_Config.Channels, "RGB channel name:", m_Config.RgbChannel, newChannel => { m_Config.RgbChannel = newChannel; });
                    break;
                case PointCloud2VisualizerSettings.ColorMode.SeparateRGB:
                    CreateNewDropdown(m_Config.Channels, "R channel name:", m_Config.RChannel, newChannel => { m_Config.RChannel = newChannel; });
                    CreateMinMaxEditor("R channel min", "Max", m_Config.RRange);

                    CreateNewDropdown(m_Config.Channels, "G channel name:", m_Config.GChannel, newChannel => { m_Config.GChannel = newChannel; });
                    CreateMinMaxEditor("G channel min", "Max", m_Config.GRange);

                    CreateNewDropdown(m_Config.Channels, "B channel name:", m_Config.BChannel, newChannel => { m_Config.BChannel = newChannel; });
                    CreateMinMaxEditor("B channel min", "Max", m_Config.BRange);
                    break;
            }
        }
    }
}
