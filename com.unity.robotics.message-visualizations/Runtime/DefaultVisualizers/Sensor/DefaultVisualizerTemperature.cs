using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerTemperature : GuiVisualFactory<TemperatureMsg>
{
    public override Action CreateGUI(TemperatureMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Temperature: {message.temperature} (ºC)\nVariance: {message.variance}");
        };
    }
}
