using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class LaserScanDefaultVisualizer : DrawingVisualizerWithSettings<LaserScanMsg, LaserScanVisualizerSettings>
{
    public override string DefaultScriptableObjectPath => "VisualizerSettings/LaserScanVisualizerSettings";
}
