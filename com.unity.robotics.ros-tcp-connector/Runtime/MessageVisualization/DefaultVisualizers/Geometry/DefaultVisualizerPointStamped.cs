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
        public float radius = 0.01f;

        public override void Draw(MPointStamped message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
            MessageVisualizations.Draw<FLU>(drawing, message.point, color, label, radius);
        }

        public override Action CreateGUI(MPointStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(message.header);
            MessageVisualizations.GUI(meta.topic, message.point);
        };
    }
}