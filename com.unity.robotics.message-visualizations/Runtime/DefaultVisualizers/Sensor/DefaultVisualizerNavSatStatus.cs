using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerNavSatStatus : VisualFactory<MNavSatStatus>
{
    public override Action CreateGUI(MNavSatStatus message, MessageMetadata meta) => () =>
    {
        message.GUI();
    };
}
