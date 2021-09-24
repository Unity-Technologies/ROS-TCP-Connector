using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class JoyDefaultVisualizer : GuiVisualizer<JoyMsg>
{
    [SerializeField]
    public JoyVisualizerSettings m_Settings;

    public override Action CreateGUI(JoyMsg message, MessageMetadata meta)
    {
        return m_Settings.CreateGUI(message, meta);
    }
}
