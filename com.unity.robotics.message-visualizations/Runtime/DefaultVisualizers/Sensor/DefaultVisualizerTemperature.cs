using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerTemperature : GuiVisualFactory<MTemperature>
{
    public override Action CreateGUI(MTemperature message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Temperature: {message.temperature} (ºC)\nVariance: {message.variance}");
    };
}
