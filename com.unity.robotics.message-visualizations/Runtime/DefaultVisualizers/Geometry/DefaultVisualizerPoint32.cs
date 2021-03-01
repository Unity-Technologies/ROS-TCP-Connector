using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPoint32 : BasicVisualizer<MPoint32>
    {
        [SerializeField]
        float m_Radius = 0.01f;

        public override void Draw(DebugDraw.Drawing drawing, MPoint32 message, MessageMetadata meta, Color color, string label)
        {
            MessageVisualizations.Draw<FLU>(drawing, message, color, label, m_Radius);
        }

        public override System.Action CreateGUI(MPoint32 message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(message);
        };
    }
}