using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJoyFeedback : GuiVisualFactory<JoyFeedbackMsg>
{
    public override Action CreateGUI(JoyFeedbackMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
