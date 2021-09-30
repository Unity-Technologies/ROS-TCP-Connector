using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class RelativeHumidityDefaultVisualizer : GuiVisualizer<RelativeHumidityMsg>
{
    public override Action CreateGUI(RelativeHumidityMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Relative Humidity: {message.relative_humidity}\nVariance: {message.variance}");
        };
    }
}
