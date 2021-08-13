using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTwistWithCovarianceStamped : DrawingVisualFactory<TwistWithCovarianceStampedMsg>
    {
        public float thickness = 0.01f;
        public float lengthScale = 1.0f;
        public float sphereRadius = 1.0f;
        public GameObject origin;
        [SerializeField]
        Color m_Color;
        bool m_ViewCovariance;

        public override void Draw(BasicDrawing drawing, TwistWithCovarianceStampedMsg message, MessageMetadata meta)
        {
            var orig = origin == null ? Vector3.zero : origin.transform.position;
            DefaultVisualizerTwistWithCovariance.Draw<FLU>(message.twist, drawing, SelectColor(m_Color, meta), orig, lengthScale, sphereRadius, thickness);
        }

        public override Action CreateGUI(TwistWithCovarianceStampedMsg message, MessageMetadata meta) => () =>
        {
            message.header.GUI();
            message.twist.twist.GUI();
            MessageVisualizations.GUIGrid(message.twist.covariance, 6, ref m_ViewCovariance);
        };
    }
}
