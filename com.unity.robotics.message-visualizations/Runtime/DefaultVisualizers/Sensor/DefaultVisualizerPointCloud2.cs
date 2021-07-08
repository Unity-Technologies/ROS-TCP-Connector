using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPointCloud2 : SettingsBasedVisualFactory<PointCloud2Msg, PointCloud2VisualizerSettings>
{
}
