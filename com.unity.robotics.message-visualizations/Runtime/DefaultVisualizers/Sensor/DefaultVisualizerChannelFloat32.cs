using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerChannelFloat32 : BasicVisualizer<MChannelFloat32>
{
    public override Action CreateGUI(MChannelFloat32 message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI();
    };
}
