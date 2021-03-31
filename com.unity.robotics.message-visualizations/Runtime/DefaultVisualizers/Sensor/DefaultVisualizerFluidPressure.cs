using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerFluidPressure : BasicVisualizer<MFluidPressure>
{
    public override Action CreateGUI(MFluidPressure message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI();
    };
}
