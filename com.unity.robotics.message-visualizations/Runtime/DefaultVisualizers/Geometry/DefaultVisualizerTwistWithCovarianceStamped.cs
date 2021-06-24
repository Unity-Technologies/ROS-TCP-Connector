using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTwistWithCovarianceStamped : DrawingVisualFactory<MTwistWithCovarianceStamped>
    {
        public float thickness = 0.01f;
        public float lengthScale = 1.0f;
        public float sphereRadius = 1.0f;
        public GameObject origin;
        [SerializeField]
        Color m_Color;
        bool m_ViewCovariance;

        public override void Draw(BasicDrawing drawing, MTwistWithCovarianceStamped message, MessageMetadata meta)
        {
            message.twist.twist.Draw<FLU>(drawing, SelectColor(m_Color, meta), origin.transform.position, lengthScale, sphereRadius, thickness);
        }

        public override Action CreateGUI(MTwistWithCovarianceStamped message, MessageMetadata meta) => () =>
        {
            message.header.GUI();
            message.twist.twist.GUI();
            MessageVisualizations.GUIGrid(message.twist.covariance, 6, ref m_ViewCovariance);
        };
    }
}
