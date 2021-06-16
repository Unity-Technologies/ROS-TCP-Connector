using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPolygonStamped : DrawingVisualFactory<MPolygonStamped>
    {
        [SerializeField]
        float m_Thickness = 0.01f;
        [SerializeField]
        Color m_Color;

        public override void Draw(BasicDrawing drawing, MPolygonStamped message, MessageMetadata meta)
        {
            message.polygon.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Thickness);
        }

        public override Action CreateGUI(MPolygonStamped message, MessageMetadata meta) => () =>
        {
            message.header.GUI();
            message.polygon.GUI();
        };
    }
}