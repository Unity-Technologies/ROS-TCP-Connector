using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using MTransform = RosMessageTypes.Geometry.Transform;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTransform : BasicVisualizer<MTransform>
    {
        public float size = 0.1f;

        public override void Draw(MTransform msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
            MessageVisualizations.Draw<FLU>(drawing, msg, size);
            drawing.DrawLabel(label, msg.translation.From<FLU>(), color, size);
        }

        public override System.Action CreateGUI(MTransform msg, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(label, msg);
        };
    }
}
