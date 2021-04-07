using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerRange : BasicVisualizer<MRange>
{
    public override Action CreateGUI(MRange message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI();
    };
}
