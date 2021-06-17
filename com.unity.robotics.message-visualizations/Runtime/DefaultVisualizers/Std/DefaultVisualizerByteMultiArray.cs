using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerByteMultiArray : GuiVisualFactory<MByteMultiArray>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(MByteMultiArray message, MessageMetadata meta) => () =>
    {
        message.layout.GUIMultiArray(message.data, ref m_Tabulate);
    };
}
