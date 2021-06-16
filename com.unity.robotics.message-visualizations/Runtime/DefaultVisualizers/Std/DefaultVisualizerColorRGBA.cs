using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerColorRGBA : GuiVisualFactory<MColorRGBA>
{
    public override Action CreateGUI(MColorRGBA message, MessageMetadata meta) => () =>
    {
        message.GUI();
    };
}
