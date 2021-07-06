using System;
using UnityEngine;

[CreateAssetMenu(fileName = "MultiEchoLaserScanVisualizerSettings", menuName = "MessageVisualizations/Sensor/MultiEchoLaserScan", order = 1)]
public class MultiEchoLaserScanVisualizerSettings : ScriptableObject
{
    public bool m_UseIntensitySize;
    public float m_PointRadius = 0.05f;
    public float[] m_SizeRange = { 0, 100 };
}
