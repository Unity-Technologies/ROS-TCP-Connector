using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class JoyFeedbackArrayDefaultVisualizer : GuiVisualizer<JoyFeedbackArrayMsg>
{
    public override Action CreateGUI(JoyFeedbackArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            foreach (JoyFeedbackMsg m in message.array) m.GUI();
        };
    }
}
