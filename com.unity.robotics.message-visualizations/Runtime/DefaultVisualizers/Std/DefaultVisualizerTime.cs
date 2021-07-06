using System;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerTime : GuiVisualFactory<TimeMsg>
{
    public override Action CreateGUI(TimeMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
