using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerTime : BasicHudOnlyVisualizer<MTime>
{
    public override Action CreateGUI(MTime message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI();
    };
}
