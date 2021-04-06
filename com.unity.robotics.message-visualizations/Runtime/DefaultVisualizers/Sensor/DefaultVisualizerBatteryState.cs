using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerBatteryState : BasicVisualizer<MBatteryState>
{
    public override Action CreateGUI(MBatteryState message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI();
    };
}
