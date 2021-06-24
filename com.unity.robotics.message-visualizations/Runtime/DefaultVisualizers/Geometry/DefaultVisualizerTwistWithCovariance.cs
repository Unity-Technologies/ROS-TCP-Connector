using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTwistWithCovariance : DrawingVisualFactory<MTwistWithCovariance>
    {
        public float thickness = 0.01f;
        public float lengthScale = 1.0f;
        public float sphereRadius = 1.0f;
        public GameObject origin;
        [SerializeField]
        Color m_Color;
        bool m_ViewCovariance;

        public override void Draw(BasicDrawing drawing, MTwistWithCovariance message, MessageMetadata meta)
        {
            message.twist.Draw<FLU>(drawing, SelectColor(m_Color, meta), origin.transform.position, lengthScale, sphereRadius, thickness);
        }

        public override Action CreateGUI(MTwistWithCovariance message, MessageMetadata meta) => () =>
        {
            message.twist.GUI();
            MessageVisualizations.GUIGrid(message.covariance, 6, ref m_ViewCovariance);
        };
    }
}
