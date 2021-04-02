using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerImage : BasicVisualizer<MImage>
{
    public override Action CreateGUI(MImage message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI();
    };
}
