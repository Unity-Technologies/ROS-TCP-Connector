using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTransform : BasicVisualizer<MTransform>
    {
        public float size = 0.1f;

        public override void Draw(MTransform message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
            MessageVisualizations.Draw<FLU>(drawing, message, size);
            drawing.DrawLabel(label, message.translation.From<FLU>(), color, size);
        }

        public override System.Action CreateGUI(MTransform message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(label, message);
        };
    }
}
