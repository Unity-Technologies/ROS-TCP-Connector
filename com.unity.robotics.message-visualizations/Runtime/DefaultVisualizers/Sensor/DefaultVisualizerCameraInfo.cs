using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerCameraInfo : BasicVisualizer<MCameraInfo>
{
    public Texture2D m_BaseImg;
    public override Action CreateGUI(MCameraInfo message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.GUI();
        MessageVisualizations.GUIGrid(message.K, 3, "K");
        MessageVisualizations.GUIGrid(message.R, 3, "R");
        MessageVisualizations.GUIGrid(message.P, 3, "P");
        message.roi.GUI(m_BaseImg, (int)message.height, (int)message.width);
    };
}
