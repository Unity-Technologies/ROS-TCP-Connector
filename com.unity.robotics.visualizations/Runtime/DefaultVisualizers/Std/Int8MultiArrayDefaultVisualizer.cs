using System;
using RosMessageTypes.Std;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class Int8MultiArrayDefaultVisualizer : GuiVisualizer<Int8MultiArrayMsg>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(Int8MultiArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.layout.GUIMultiArray(message.data, ref m_Tabulate);
        };
    }
}
