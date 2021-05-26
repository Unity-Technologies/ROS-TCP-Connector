using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerJoyFeedbackArray : BasicVisualizer<MJoyFeedbackArray>
{
    public override Action CreateGUI(MJoyFeedbackArray message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        foreach (MJoyFeedback m in message.array)
        {
            string type = ((JoyFeedbackTypes)m.type).ToString().Substring("TYPE_".Length);
            m.GUI(type);
        }
    };
}
