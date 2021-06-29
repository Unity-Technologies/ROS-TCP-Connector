using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerPolygonStamped : DrawingVisualFactory<PolygonStampedMsg>
    {
        [SerializeField]
        float m_Thickness = 0.01f;
        [SerializeField]
        Color m_Color;

        public override void Draw(BasicDrawing drawing, PolygonStampedMsg message, MessageMetadata meta)
        {
            message.polygon.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Thickness);
        }

        public override Action CreateGUI(PolygonStampedMsg message, MessageMetadata meta)
        {
            return () =>
            {
                message.header.GUI();
                message.polygon.GUI();
            };
        }
    }
}
