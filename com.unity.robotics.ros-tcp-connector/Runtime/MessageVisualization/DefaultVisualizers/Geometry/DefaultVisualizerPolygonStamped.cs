using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using MPolygonStamped = RosMessageTypes.Geometry.PolygonStamped;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPolygonStamped : BasicVisualizer<MPolygonStamped>
    {
        public float thickness = 0.01f;

        public override void Draw(MPolygonStamped message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
            MessageVisualizations.Draw<FLU>(drawing, message.polygon, color, thickness);
        }

        public override Action CreateGUI(MPolygonStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(message.header);
            MessageVisualizations.GUI(message.polygon);
        };
    }
}