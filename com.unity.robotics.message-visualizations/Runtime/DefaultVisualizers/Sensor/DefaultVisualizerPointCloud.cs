using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerPointCloud : SettingsBasedVisualFactory<PointCloudMsg, PointCloudVisualizerSettings>
{
    public override string DefaultScriptableObjectPath => "VisualizerSettings/PointCloudVisualizerSettings";
}
