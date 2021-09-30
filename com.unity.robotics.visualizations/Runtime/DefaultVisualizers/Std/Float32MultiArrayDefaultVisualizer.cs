using System;
using RosMessageTypes.Std;
using Unity.Robotics.Visualizations;
using UnityEngine;

public class Float32MultiArrayDefaultVisualizer : GuiVisualizer<Float32MultiArrayMsg>
{
    [SerializeField]
    bool m_Tabulate = true;

    public override Action CreateGUI(Float32MultiArrayMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.layout.GUIMultiArray(message.data, ref m_Tabulate);
        };
    }
}
