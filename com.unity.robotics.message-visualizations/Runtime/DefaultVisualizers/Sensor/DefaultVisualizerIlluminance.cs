using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerIlluminance : GuiVisualFactory<IlluminanceMsg>
{
    public override Action CreateGUI(IlluminanceMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Illuminance: {message.illuminance} (Lux)\nVariance: {message.variance}");
        };
    }
}
