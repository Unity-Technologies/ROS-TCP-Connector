using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerQuaternionStamped : BasicVisualizer<MQuaternionStamped>
    {
        [SerializeField]
        float m_Size = 0.01f;
        [SerializeField]
        GameObject m_DrawAtPosition;

        public override void Draw(DebugDraw.Drawing drawing, MQuaternionStamped message, MessageMetadata meta, Color color, string label)
        {
            message.quaternion.Draw<FLU>(drawing, m_DrawAtPosition, m_Size);
            drawing.DrawLabel(label, transform.position, color, m_Size);
        }

        public override Action CreateGUI(MQuaternionStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            message.header.GUI();
            message.quaternion.GUI();
        };
    }
}