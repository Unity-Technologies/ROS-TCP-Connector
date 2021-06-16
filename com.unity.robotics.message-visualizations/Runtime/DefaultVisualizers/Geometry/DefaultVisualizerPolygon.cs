using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPolygon : DrawingVisualFactory<MPolygon>
    {
        [SerializeField]
        float m_Thickness = 0.1f;
        [SerializeField]
        Color m_Color;

        public override void Draw(BasicDrawing drawing, MPolygon message, MessageMetadata meta)
        {
            message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Thickness);
        }

        public override System.Action CreateGUI(MPolygon message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}