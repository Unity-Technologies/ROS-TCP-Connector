using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerRegionOfInterest : BasicVisualizer<MRegionOfInterest>
{
    public Texture2D m_BaseImg;
    public int m_Height;
    public int m_Width;

    public override Action CreateGUI(MRegionOfInterest message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI(m_BaseImg, m_Height, m_Width);
    };
}
