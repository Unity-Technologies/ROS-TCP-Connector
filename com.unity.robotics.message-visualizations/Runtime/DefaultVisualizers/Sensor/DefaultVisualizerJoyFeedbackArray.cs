using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJoyFeedbackArray : BasicVisualFactory<MJoyFeedbackArray>
{
    public override Action CreateGUI(MJoyFeedbackArray message, MessageMetadata meta) => () =>
    {
        foreach (MJoyFeedback m in message.array)
        {
            m.GUI();
        }
    };
}
