using System;
using RosMessageTypes.Sensor;
using UnityEngine;

[CreateAssetMenu(fileName = "PointCloudVisualizerSettings", menuName = "MessageVisualizations/Sensor/PointCloud", order = 1)]
public class PointCloudVisualizerSettings : ScriptableObject
{
    public ColorMode colorMode;
    public ChannelFloat32Msg[] channels;
    public string m_RgbChannel = "";
    public string m_RChannel = "";
    public string m_GChannel = "";
    public string m_BChannel = "";
    public string m_SizeChannel = "";

    public float[] m_RgbRange = { 0, 100 };
    public float[] m_RRange = { 0, 100 };
    public float[] m_GRange = { 0, 100 };
    public float[] m_BRange = { 0, 100 };
    public float[] m_SizeRange = { 0, 100 };
    public float m_Size = 0.05f;

    public bool m_UseRgbChannel;
    public bool m_UseSeparateRgb = true;
    public bool m_UseSizeChannel;
}
