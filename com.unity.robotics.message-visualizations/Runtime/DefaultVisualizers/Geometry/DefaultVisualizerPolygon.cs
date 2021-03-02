using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPolygon : BasicVisualizer<MPolygon>
    {
        [SerializeField]
        float m_Thickness = 0.1f;

        public override void Draw(DebugDraw.Drawing drawing, MPolygon message, MessageMetadata meta, Color color, string label)
        {
            message.Draw<FLU>(drawing, color, m_Thickness);
        }

        public override System.Action CreateGUI(MPolygon message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            message.GUI();
        };
    }
}