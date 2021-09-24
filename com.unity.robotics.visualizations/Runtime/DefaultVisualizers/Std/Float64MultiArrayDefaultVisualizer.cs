using System;
using RosMessageTypes.Std;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class Float64MultiArrayDefaultVisualizer : GuiVisualizer<Float64MultiArrayMsg>
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
