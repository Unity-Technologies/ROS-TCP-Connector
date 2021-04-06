using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJoyFeedbackArray : BasicVisualizer<MJoyFeedbackArray>
{
    public override Action CreateGUI(MJoyFeedbackArray message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI();
    };
}
