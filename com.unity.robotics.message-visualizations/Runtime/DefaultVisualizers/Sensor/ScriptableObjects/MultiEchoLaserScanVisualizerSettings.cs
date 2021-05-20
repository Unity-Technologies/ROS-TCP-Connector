using RosMessageTypes.Sensor;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

[CreateAssetMenu(fileName = "MultiEchoLaserScanVisualizerSettings", menuName = "MessageVisualizations/Sensor/MultiEchoLaserScan", order = 1)]
public class MultiEchoLaserScanVisualizerSettings : ScriptableObject
{
    public bool m_UseIntensitySize = false;
    public float m_PointRadius = 0.05f;
    public float[] m_SizeRange = new float[] { 0, 100 };
}
