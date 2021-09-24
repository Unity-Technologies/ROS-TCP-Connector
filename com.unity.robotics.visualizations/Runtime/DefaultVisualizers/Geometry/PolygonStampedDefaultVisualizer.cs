using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class PolygonStampedDefaultVisualizer : DrawingVisualizer<PolygonStampedMsg>
    {
        [SerializeField]
        float m_Thickness = 0.01f;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        TFTrackingSettings m_TFTrackingSettings;

        public override void Draw(Drawing3d drawing, PolygonStampedMsg message, MessageMetadata meta)
        {
            drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
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
