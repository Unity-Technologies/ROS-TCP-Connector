using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class TimeReferenceDefaultVisualizer : GuiVisualizer<TimeReferenceMsg>
{
    public override Action CreateGUI(TimeReferenceMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Time reference:{message.time_ref.ToTimestampString()}\nSource: {message.source}");
        };
    }
}
