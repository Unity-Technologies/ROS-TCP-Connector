using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class WrenchStampedDefaultVisualizer : DrawingVisualizer<WrenchStampedMsg>
    {
        public float thickness = 0.01f;
        public float lengthScale = 1.0f;
        public float sphereRadius = 1.0f;
        public GameObject origin;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        TFTrackingSettings m_TFTrackingSettings;

        public override void Draw(Drawing3d drawing, WrenchStampedMsg message, MessageMetadata meta)
        {
            drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
            WrenchDefaultVisualizer.Draw<FLU>(message.wrench, drawing, SelectColor(m_Color, meta), origin.transform.position, lengthScale, sphereRadius, thickness);
        }

        public override Action CreateGUI(WrenchStampedMsg message, MessageMetadata meta) => () =>
        {
            message.header.GUI();
            message.wrench.GUI();
        };
    }
}
