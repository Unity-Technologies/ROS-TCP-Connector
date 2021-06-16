using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJoyFeedback : GuiVisualFactory<MJoyFeedback>
{
    public override Action CreateGUI(MJoyFeedback message, MessageMetadata meta) => () =>
    {
        message.GUI();
    };
}