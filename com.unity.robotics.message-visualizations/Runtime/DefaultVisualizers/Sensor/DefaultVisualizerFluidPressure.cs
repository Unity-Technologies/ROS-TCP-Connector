using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerFluidPressure : BasicVisualFactory<MFluidPressure>
{
    public override Action CreateGUI(MFluidPressure message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Fluid Pressure: {message.fluid_pressure} (Pascals)\nVariance: {message.variance}");
    };
}
