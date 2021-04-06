using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJoy : BasicVisualizer<MJoy>
{
    [SerializeField]
    int m_Layout = 0;

    public override Action CreateGUI(MJoy message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI(ref m_Layout);
    };
}
