using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPolygon : BasicVisualizer<MPolygon>
    {
        public float thickness = 0.1f;

        public override void Draw(MPolygon message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
            MessageVisualizations.Draw<FLU>(drawing, message, color, thickness);
        }

        public override System.Action CreateGUI(MPolygon message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            foreach (MPoint32 p in message.points)
                MessageVisualizations.GUI(label, p);
        };
    }
}