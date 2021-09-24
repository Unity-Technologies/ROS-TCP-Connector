using System;
using System.Linq;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PointCloudDefaultVisualizer : DrawingVisualizerWithSettings<PointCloudMsg, PointCloudVisualizerSettings>
{
    public override string DefaultScriptableObjectPath => "VisualizerSettings/PointCloudVisualizerSettings";
}
