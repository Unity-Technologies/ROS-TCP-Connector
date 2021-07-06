using System;
using UnityEngine;

[CreateAssetMenu(fileName = "MultiEchoLaserScanVisualizerSettings", menuName = "MessageVisualizations/Sensor/MultiEchoLaserScan", order = 1)]
public class MultiEchoLaserScanVisualizerSettings : ScriptableObject
{
    public string topic;
    public bool useIntensitySize;
    public float pointRadius = 0.05f;
    public float[] sizeRange = { 0, 100 };
}
