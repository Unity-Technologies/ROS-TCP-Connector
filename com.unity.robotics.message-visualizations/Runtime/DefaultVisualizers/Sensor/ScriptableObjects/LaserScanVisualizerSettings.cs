using System;
using UnityEngine;

[CreateAssetMenu(fileName = "LaserScanVisualizerSettings", menuName = "MessageVisualizations/Sensor/LaserScan", order = 1)]
public class LaserScanVisualizerSettings : ScriptableObject
{
    public string topic;
    public bool useIntensitySize;
    public float pointRadius = 0.05f;
    public float[] sizeRange = { 0, 100 };
}
