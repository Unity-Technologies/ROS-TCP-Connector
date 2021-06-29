using System;
using RosMessageTypes.Sensor;
using UnityEngine;

[CreateAssetMenu(fileName = "PointCloud2VisualizerSettings", menuName = "MessageVisualizations/Sensor/PointCloud2", order = 1)]
public class PointCloud2VisualizerSettings : ScriptableObject
{
    public ColorMode colorMode;
    public PointFieldMsg[] channels;

    public string m_XChannel = "x";
    public string m_YChannel = "y";
    public string m_ZChannel = "z";
    public string m_RgbChannel = "x";
    public string m_RChannel = "x";
    public string m_GChannel = "y";
    public string m_BChannel = "z";
    public string m_SizeChannel = "x";

    public float[] m_RgbRange = { 0, 31 };
    public float[] m_RRange = { -100, 100 };
    public float[] m_GRange = { -100, 100 };
    public float[] m_BRange = { -100, 100 };
    public float[] m_SizeRange = { 0, 100 };
    public float m_Size = 0.01f;

    public bool m_UseRgbChannel;
    public bool m_UseSizeChannel;
}
