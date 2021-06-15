using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerRelativeHumidity : BasicVisualFactory<MRelativeHumidity>
{
    public override Action CreateGUI(MRelativeHumidity message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Relative Humidity: {message.relative_humidity}\nVariance: {message.variance}");
    };
}
