using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class NavSatStatusDefaultVisualizer : GuiVisualizer<NavSatStatusMsg>
{
    public override Action CreateGUI(NavSatStatusMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
