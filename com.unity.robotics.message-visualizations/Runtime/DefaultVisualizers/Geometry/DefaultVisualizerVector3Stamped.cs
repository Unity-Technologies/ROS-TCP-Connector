using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerVector3Stamped : BasicVisualizer<MVector3Stamped>
    {
        [SerializeField]
        float m_Radius = 0.01f;

        public override void Draw(DebugDraw.Drawing drawing, MVector3Stamped message, MessageMetadata meta, Color color, string label)
        {
            MessageVisualizations.Draw<FLU>(drawing, message.vector, color, label, m_Radius);
        }

        public override System.Action CreateGUI(MVector3Stamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(message.header);
            MessageVisualizations.GUI(SelectLabel(meta), message.vector);
        };
    }
}