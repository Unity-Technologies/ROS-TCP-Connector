using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerTemperature : BasicVisualizer<MTemperature>
{
    public override Action CreateGUI(MTemperature message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Temperature: {message.temperature} (ºC)\nVariance: {message.variance}");
    };
}
