using RosMessageTypes.Sensor;
using System;
using Unity.Robotics.Visualizations;

public class JoyFeedbackDefaultVisualizer : GuiVisualizer<JoyFeedbackMsg>
{
    public override Action CreateGUI(JoyFeedbackMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
