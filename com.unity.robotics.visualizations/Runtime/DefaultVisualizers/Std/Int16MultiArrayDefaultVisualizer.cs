using System;
using RosMessageTypes.Std;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class Int16MultiArrayDefaultVisualizer : GuiVisualizer<Int16MultiArrayMsg>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(Int16MultiArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.layout.GUIMultiArray(message.data, ref m_Tabulate);
        };
    }
}
