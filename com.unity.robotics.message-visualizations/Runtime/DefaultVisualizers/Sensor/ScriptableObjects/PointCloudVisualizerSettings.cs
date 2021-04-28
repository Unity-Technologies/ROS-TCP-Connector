using RosMessageTypes.Sensor;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

[CreateAssetMenu(fileName = "PointCloudVisualizerSettings", menuName = "MessageVisualizations/Sensor/PointCloud", order = 1)]
public class PointCloudVisualizerSettings : ScriptableObject
{
    public ColorMode colorMode;
    public MChannelFloat32[] channels;
    public string m_RgbChannel = "";
    public string m_RChannel = "";
    public string m_GChannel = "";
    public string m_BChannel = "";
    public string m_SizeChannel = "";

    public float[] m_RgbRange = new float[] { 0, 100 };
    public float[] m_RRange = new float[] { 0, 100 };
    public float[] m_GRange = new float[] { 0, 100 };
    public float[] m_BRange = new float[] { 0, 100 };
    public float[] m_SizeRange = new float[] { 0, 100 };
    public float m_Size = 0.05f;

    public bool m_UseRgbChannel = false;
    public bool m_UseSeparateRgb = true;
    public bool m_UseSizeChannel = false;
}
