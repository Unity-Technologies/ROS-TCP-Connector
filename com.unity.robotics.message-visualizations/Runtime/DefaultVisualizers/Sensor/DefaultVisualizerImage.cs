using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerImage : BasicVisualizer<MImage>
{
    bool m_ConvertFromBGR = true;
    bool m_FlipY = false;

    Texture2D m_Tex;

    public override Action CreateGUI(MImage message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI(ref m_Tex, ref m_ConvertFromBGR, ref m_FlipY);
    };
}
