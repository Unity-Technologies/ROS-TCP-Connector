using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerUInt8MultiArray : GuiVisualFactory<MUInt8MultiArray>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(MUInt8MultiArray message, MessageMetadata meta) => () =>
    {
        message.layout.GUIMultiArray(message.data, ref m_Tabulate);
    };
}
