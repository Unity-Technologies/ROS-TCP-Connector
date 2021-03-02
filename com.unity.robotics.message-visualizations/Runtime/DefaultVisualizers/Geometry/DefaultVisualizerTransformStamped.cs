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
        [SerializeField]
        float m_Size = 0.1f;

        public override void Draw(DebugDraw.Drawing drawing, MTransformStamped message, MessageMetadata meta, Color color, string label)
        {
            message.transform.Draw<FLU>(drawing, m_Size);
            drawing.DrawLabel(label, message.transform.translation.From<FLU>(), color, m_Size);
        }

        public override System.Action CreateGUI(MTransformStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            message.header.GUI();
            message.transform.GUI();
        };
    }
}
