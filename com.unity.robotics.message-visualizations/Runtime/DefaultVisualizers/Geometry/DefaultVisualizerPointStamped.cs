using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPointStamped : BasicVisualizer<MPointStamped>
    {
        [SerializeField]
        float m_Radius = 0.01f;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        string m_Label;

        public override void Draw(BasicDrawing drawing, MPointStamped message, MessageMetadata meta)
        {
            message.point.Draw<FLU>(drawing, SelectColor(m_Color, meta), SelectLabel(m_Label, meta), m_Radius);
        }

        public override Action CreateGUI(MPointStamped message, MessageMetadata meta, BasicDrawing drawing) => () =>
        {
            message.header.GUI();
            message.point.GUI();
        };
    }
}