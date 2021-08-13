using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class PolygonStampedDefaultVisualizer : DrawingVisualizer<PolygonStampedMsg>
    {
        [SerializeField]
        float m_Thickness = 0.01f;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        TFTrackingType m_TFTrackingType;

        public override void Draw(BasicDrawing drawing, PolygonStampedMsg message, MessageMetadata meta)
        {
            drawing.SetTFTrackingType(m_TFTrackingType, message.header);
            PolygonDefaultVisualizer.Draw<FLU>(message.polygon, drawing, SelectColor(m_Color, meta), m_Thickness);
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
