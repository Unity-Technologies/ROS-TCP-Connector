using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPoint : BasicVisualizer<MPoint>
    {
        [SerializeField]
        float m_Radius = 0.01f;

        public override void Draw(BasicDrawing drawing, MPoint message, MessageMetadata meta, Color color, string label)
        {
            message.Draw<FLU>(drawing, color, label, m_Radius);
        }

        public override System.Action CreateGUI(MPoint message, MessageMetadata meta, BasicDrawing drawing) => () =>
        {
            message.GUI();
        };
    }
}