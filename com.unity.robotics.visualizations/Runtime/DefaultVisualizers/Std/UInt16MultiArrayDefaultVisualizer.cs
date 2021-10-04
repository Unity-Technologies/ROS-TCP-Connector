using System;
using RosMessageTypes.Std;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class UInt16MultiArrayDefaultVisualizer : GuiVisualizer<UInt16MultiArrayMsg>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(UInt16MultiArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.layout.GUIMultiArray(message.data, ref m_Tabulate);
        };
    }
}
