using System;
using UnityEngine;

[CreateAssetMenu(fileName = "LaserScanVisualizerSettings", menuName = "MessageVisualizations/Sensor/LaserScan", order = 1)]
public class LaserScanVisualizerSettings : ScriptableObject
{
    public bool m_UseIntensitySize;
    public float m_PointRadius = 0.05f;
    public float[] m_SizeRange = { 0, 100 };
}
