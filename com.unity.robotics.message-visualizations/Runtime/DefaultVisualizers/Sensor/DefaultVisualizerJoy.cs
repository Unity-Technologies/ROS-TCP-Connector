using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerJoy : BasicVisualizer<MJoy>
{
    int m_Layout = 0;
    string[] m_SelStrings = {"DS4", "X360 Windows", "X360 Linux", "X360 (Wired)", "F710"};

    public override Action CreateGUI(MJoy message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.GUI(ref m_Layout, m_SelStrings);
    };
}
