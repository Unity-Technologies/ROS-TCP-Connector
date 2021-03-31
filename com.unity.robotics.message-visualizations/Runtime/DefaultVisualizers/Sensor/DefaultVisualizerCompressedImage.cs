using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerCompressedImage : BasicVisualizer<MCompressedImage>
{
    public override Action CreateGUI(MCompressedImage message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI();
    };
}
