using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class FluidPressureDefaultVisualizer : GuiVisualizer<FluidPressureMsg>
{
    public override Action CreateGUI(FluidPressureMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Fluid Pressure: {message.fluid_pressure} (Pascals)\nVariance: {message.variance}");
        };
    }
}
