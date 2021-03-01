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
        [SerializeField]
        float m_Size = 0.1f;

        public override void Draw(DebugDraw.Drawing drawing, MTransform message, MessageMetadata meta, Color color, string label)
        {
            MessageVisualizations.Draw<FLU>(drawing, message, m_Size);
            drawing.DrawLabel(label, message.translation.From<FLU>(), color, m_Size);
        }

        public override System.Action CreateGUI(MTransform message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(message);
        };
    }
}
