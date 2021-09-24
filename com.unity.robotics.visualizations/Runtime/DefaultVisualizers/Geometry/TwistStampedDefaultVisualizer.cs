using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class TwistStampedDefaultVisualizer : DrawingVisualizer<TwistStampedMsg>
    {
        public float thickness = 0.01f;
        public float lengthScale = 1.0f;
        public float sphereRadius = 1.0f;
        public GameObject origin;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        TFTrackingSettings m_TFTrackingSettings;

        public override void Draw(Drawing3d drawing, TwistStampedMsg message, MessageMetadata meta)
        {
            drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
            var orig = origin == null ? Vector3.zero : origin.transform.position;
            TwistDefaultVisualizer.Draw<FLU>(message.twist, drawing, SelectColor(m_Color, meta), orig, lengthScale, sphereRadius, thickness);
        }

        public override Action CreateGUI(TwistStampedMsg message, MessageMetadata meta) => () =>
        {
            message.header.GUI();
            message.twist.GUI();
        };
    }
}
