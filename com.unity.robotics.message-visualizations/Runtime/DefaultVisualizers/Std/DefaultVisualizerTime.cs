using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerTime : GuiVisualFactory<MTime>
{
    public override Action CreateGUI(MTime message, MessageMetadata meta) => () =>
    {
        message.GUI();
    };
}
