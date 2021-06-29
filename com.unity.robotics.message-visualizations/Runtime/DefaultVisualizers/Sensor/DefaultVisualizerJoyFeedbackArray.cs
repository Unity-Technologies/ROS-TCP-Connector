using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJoyFeedbackArray : GuiVisualFactory<JoyFeedbackArrayMsg>
{
    public override Action CreateGUI(JoyFeedbackArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            foreach (MJoyFeedback m in message.array) m.GUI();
        };
    }
}
