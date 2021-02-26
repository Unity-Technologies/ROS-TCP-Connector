using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTransformStamped : BasicVisualizer<MTransformStamped>
    {
        public float size = 0.1f;

        public override void Draw(MTransformStamped message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
            MessageVisualizations.Draw<FLU>(drawing, message.transform, size);
            drawing.DrawLabel(label, message.transform.translation.From<FLU>(), color, size);
        }

        public override System.Action CreateGUI(MTransformStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(message.header);
            MessageVisualizations.GUI(message.transform);
        };
    }
}
