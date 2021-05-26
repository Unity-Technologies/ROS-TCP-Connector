using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerJoyFeedback : BasicVisualizer<MJoyFeedback>
{
    public override Action CreateGUI(MJoyFeedback message, MessageMetadata meta, BasicDrawing drawing)
    {
        string type = ((JoyFeedbackTypes)message.type).ToString().Substring("TYPE_".Length);
        return () =>
        {
            message.GUI(type);
        };
    }
}
