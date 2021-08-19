using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
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
        TFTrackingType m_TFTrackingType;

        public override void Draw(BasicDrawing drawing, WrenchStampedMsg message, MessageMetadata meta)
        {
            drawing.SetTFTrackingType(m_TFTrackingType, message.header);
            WrenchDefaultVisualizer.Draw<FLU>(message.wrench, drawing, SelectColor(m_Color, meta), origin.transform.position, lengthScale, sphereRadius, thickness);
        }

        public override Action CreateGUI(WrenchStampedMsg message, MessageMetadata meta) => () =>
        {
            message.header.GUI();
            message.wrench.GUI();
        };
    }
}
