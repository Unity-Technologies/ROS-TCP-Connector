using RosMessageTypes.Sensor;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

[CreateAssetMenu(fileName = "PointCloud2VisualizerSettings", menuName = "MessageVisualizations/Sensor/PointCloud2", order = 1)]
public class PointCloud2VisualizerSettings : ScriptableObject
{
    public ColorMode colorMode;
    public MPointField[] channels;

    public string m_XChannel = "x";
    public string m_YChannel = "y";
    public string m_ZChannel = "z";
    public string m_RgbChannel = "x";
    public string m_RChannel = "x";
    public string m_GChannel = "y";
    public string m_BChannel = "z";
    public string m_SizeChannel = "x";

    public float[] m_RgbRange = new float[] { 0, 31 };
    public float[] m_RRange = new float[] { -100, 100 };
    public float[] m_GRange = new float[] { -100, 100 };
    public float[] m_BRange = new float[] { -100, 100 };
    public float[] m_SizeRange = new float[] { 0, 100 };
    public float m_Size = 0.01f;

    public bool m_UseRgbChannel = false;
    public bool m_UseSizeChannel = false;
}
