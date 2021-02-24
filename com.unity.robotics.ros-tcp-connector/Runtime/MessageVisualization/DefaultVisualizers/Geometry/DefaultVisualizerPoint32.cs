using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using MPoint32 = RosMessageTypes.Geometry.Point32;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPoint32 : BasicVisualizer<MPoint32>
    {
        public float size = 0.01f;

        public override void Draw(MPoint32 message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
            MessageVisualizations.Draw<FLU>(drawing, message, color, label, size);
        }

        public override System.Action CreateGUI(MPoint32 message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(label, message);
        };
    }
}