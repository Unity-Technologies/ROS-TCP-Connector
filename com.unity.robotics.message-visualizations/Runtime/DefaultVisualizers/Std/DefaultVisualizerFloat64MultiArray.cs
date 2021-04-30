using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerFloat64MultiArray : BasicHudOnlyVisualizer<MFloat64MultiArray>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(MFloat64MultiArray message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.layout.GUIMultiArray(message.data, ref m_Tabulate);
    };

}
