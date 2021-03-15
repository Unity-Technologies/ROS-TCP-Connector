using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPolygonStamped : BasicVisualizer<MPolygonStamped>
    {
        [SerializeField]
        float m_Thickness = 0.01f;

        public override void Draw(BasicDrawing drawing, MPolygonStamped message, MessageMetadata meta, Color color, string label)
        {
            message.polygon.Draw<FLU>(drawing, color, m_Thickness);
        }

        public override Action CreateGUI(MPolygonStamped message, MessageMetadata meta, BasicDrawing drawing) => () =>
        {
            message.header.GUI();
            message.polygon.GUI();
        };
    }
}