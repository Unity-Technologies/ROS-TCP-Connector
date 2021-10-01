using System;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class TimeDefaultVisualizer : GuiVisualizer<TimeMsg>
{
    public override Action CreateGUI(TimeMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
