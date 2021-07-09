using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.MessageVisualizers;
using UnityEditor;
using UnityEngine;

#if UNITY_EDITOR

[CustomEditor(typeof(DefaultVisualizerLaserScan))]
public class LaserScanEditor : SettingsBasedVisualizerEditor<LaserScanMsg, LaserScanVisualizerSettings>
{
}

#endif //UNITY_EDITOR
