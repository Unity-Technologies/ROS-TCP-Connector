using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerImage : BasicVisualizer<MImage>
{
    [SerializeField]
    bool m_convertFromBGR = true;

    Texture2D m_Tex;

    public override Action CreateGUI(MImage message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.GUI(ref m_Tex, ref m_convertFromBGR);
    };
}
