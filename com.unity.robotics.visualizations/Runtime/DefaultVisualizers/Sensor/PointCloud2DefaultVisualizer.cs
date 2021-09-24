using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PointCloud2DefaultVisualizer : DrawingVisualizerWithSettings<PointCloud2Msg, PointCloud2VisualizerSettings>
{
    public override string DefaultScriptableObjectPath => "VisualizerSettings/PointCloud2VisualizerSettings";
}
