using System;
using RosMessageTypes.Std;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerUInt32MultiArray : GuiVisualFactory<UInt32MultiArrayMsg>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(UInt32MultiArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.layout.GUIMultiArray(message.data, ref m_Tabulate);
        };
    }
}
