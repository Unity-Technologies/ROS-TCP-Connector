using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPath : BasicVisualizer<MPath>
{
    [SerializeField]
    float m_Thickness;

    public override void Draw(BasicDrawing drawing, MPath message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color, m_Thickness);
    }

    public override Action CreateGUI(MPath message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        foreach(MPoseStamped pose in message.poses)
        {
            pose.header.GUI();
            pose.pose.GUI();
        }
    };
}
