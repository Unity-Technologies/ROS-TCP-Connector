using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.MessageVisualizers;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(LaserScanDefaultVisualizer))]
public class LaserScanEditor : SettingsBasedVisualizerEditor<LaserScanMsg, LaserScanVisualizerSettings>
{
}
