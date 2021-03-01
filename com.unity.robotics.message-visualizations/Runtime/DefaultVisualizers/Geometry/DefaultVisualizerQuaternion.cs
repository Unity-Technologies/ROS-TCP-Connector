using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerQuaternion : BasicVisualizer<MQuaternion>
    {
        [SerializeField]
        float m_Size = 0.1f;
        [SerializeField]
        GameObject m_DrawAtPosition;

        public override void Draw(DebugDraw.Drawing drawing, MQuaternion message, MessageMetadata meta, Color color, string label)
        {
            MessageVisualizations.Draw<FLU>(drawing, message, m_DrawAtPosition, m_Size);
            drawing.DrawLabel(label, transform.position, color, m_Size);
        }

        public override System.Action CreateGUI(MQuaternion message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(message);
        };
    }
}