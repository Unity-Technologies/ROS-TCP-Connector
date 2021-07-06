using System;
using RosMessageTypes.Std;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerInt32MultiArray : GuiVisualFactory<Int32MultiArrayMsg>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(Int32MultiArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.layout.GUIMultiArray(message.data, ref m_Tabulate);
        };
    }
}
