using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerVector3 : BasicVisualizer<MVector3>
    {
        [SerializeField]
        float m_Radius = 0.01f;

        public override void Draw(DebugDraw.Drawing drawing, MVector3 message, MessageMetadata meta, Color color, string label)
        {
            message.Draw<FLU>(drawing, color, label, m_Radius);
        }

        public override System.Action CreateGUI(MVector3 message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            message.GUI();
        };
    }
}