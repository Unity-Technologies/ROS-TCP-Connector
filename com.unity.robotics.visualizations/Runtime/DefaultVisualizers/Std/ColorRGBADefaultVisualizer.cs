using System;
using RosMessageTypes.Std;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class ColorRGBADefaultVisualizer : GuiVisualizer<ColorRGBAMsg>
{
    public override Action CreateGUI(ColorRGBAMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
