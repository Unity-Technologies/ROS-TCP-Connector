using System;
using RosMessageTypes.Std;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerFloat64MultiArray : GuiVisualFactory<Float64MultiArrayMsg>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(Float64MultiArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.layout.GUIMultiArray(message.data, ref m_Tabulate);
        };
    }
}
