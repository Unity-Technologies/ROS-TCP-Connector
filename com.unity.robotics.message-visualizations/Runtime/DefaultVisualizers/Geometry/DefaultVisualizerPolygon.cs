using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPolygon : DrawingVisualFactory<PolygonMsg>
    {
        [SerializeField]
        float m_Thickness = 0.1f;
        [SerializeField]
        Color m_Color;

        public override void Draw(BasicDrawing drawing, PolygonMsg message, MessageMetadata meta)
        {
            message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Thickness);
        }

        public override Action CreateGUI(PolygonMsg message, MessageMetadata meta)
        {
            return () =>
            {
                message.GUI();
            };
        }
    }
}
