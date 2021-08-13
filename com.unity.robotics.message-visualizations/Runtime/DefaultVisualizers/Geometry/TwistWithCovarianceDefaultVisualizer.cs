using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class TwistWithCovarianceDefaultVisualizer : DrawingVisualizer<TwistWithCovarianceMsg>
    {
        public float thickness = 0.01f;
        public float lengthScale = 1.0f;
        public float sphereRadius = 1.0f;
        public GameObject origin;
        [SerializeField]
        Color m_Color;
        bool m_ViewCovariance;

        public override void Draw(BasicDrawing drawing, TwistWithCovarianceMsg message, MessageMetadata meta)
        {
            var orig = origin == null ? Vector3.zero : origin.transform.position;
            Draw<FLU>(message, drawing, SelectColor(m_Color, meta), orig, lengthScale, sphereRadius, thickness);
        }

        public static void Draw<C>(TwistWithCovarianceMsg message, BasicDrawing drawing, Color color, Vector3 origin, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            TwistDefaultVisualizer.Draw<FLU>(message.twist, drawing, color, origin, lengthScale, sphereRadius, thickness);
        }

        public override Action CreateGUI(TwistWithCovarianceMsg message, MessageMetadata meta) => () =>
        {
            message.twist.GUI();
            MessageVisualizationUtils.GUIGrid(message.covariance, 6, ref m_ViewCovariance);
        };
    }
}
