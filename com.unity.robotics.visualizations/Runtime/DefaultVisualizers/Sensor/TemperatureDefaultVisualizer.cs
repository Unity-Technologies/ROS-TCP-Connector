using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class TemperatureDefaultVisualizer : GuiVisualizer<TemperatureMsg>
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
