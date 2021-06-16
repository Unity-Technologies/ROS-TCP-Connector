using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPath : VisualFactory<MPath>
{
    [SerializeField]
    float m_Thickness;
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MPath message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Thickness);
    }

    public override Action CreateGUI(MPath message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        foreach (MPoseStamped pose in message.poses)
        {
            pose.header.GUI();
            pose.pose.GUI();
        }
    };
}
