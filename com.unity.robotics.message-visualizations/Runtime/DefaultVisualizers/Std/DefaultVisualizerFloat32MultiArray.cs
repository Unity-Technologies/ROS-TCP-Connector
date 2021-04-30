using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerFloat32MultiArray : BasicHudOnlyVisualizer<MFloat32MultiArray>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(MFloat32MultiArray message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.layout.GUIMultiArray(message.data, ref m_Tabulate);
    };

}
