using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerIlluminance : VisualFactory<MIlluminance>
{
    public override Action CreateGUI(MIlluminance message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Illuminance: {message.illuminance} (Lux)\nVariance: {message.variance}");
    };
}
