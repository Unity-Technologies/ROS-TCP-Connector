using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMultiEchoLaserScan : SettingsBasedVisualFactory<MultiEchoLaserScanMsg, MultiEchoLaserScanVisualizerSettings>
{
    public override string DefaultScriptableObjectPath => "VisualizerSettings/MultiEchoLaserScanVisualizerSettings";
}
