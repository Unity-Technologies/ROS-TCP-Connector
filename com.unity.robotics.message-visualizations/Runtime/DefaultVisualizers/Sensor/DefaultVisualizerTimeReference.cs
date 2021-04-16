using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerTimeReference : BasicVisualizer<MTimeReference>
{
    public override Action CreateGUI(MTimeReference message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Time reference:{message.time_ref.ToTimestampString()}\nSource: {message.source}");
    };
}
