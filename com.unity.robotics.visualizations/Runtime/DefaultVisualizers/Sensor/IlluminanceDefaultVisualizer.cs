using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class IlluminanceDefaultVisualizer : GuiVisualizer<IlluminanceMsg>
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
