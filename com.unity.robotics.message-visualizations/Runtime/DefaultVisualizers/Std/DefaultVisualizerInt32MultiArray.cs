using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerInt32MultiArray : GuiVisualFactory<MInt32MultiArray>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(MInt32MultiArray message, MessageMetadata meta) => () =>
    {
        message.layout.GUIMultiArray(message.data, ref m_Tabulate);
    };
}
