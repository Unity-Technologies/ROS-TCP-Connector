using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerUInt16MultiArray : BasicHudOnlyVisualizer<MUInt16MultiArray>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(MUInt16MultiArray message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.layout.GUIMultiArray(message.data, ref m_Tabulate);
    };
}
