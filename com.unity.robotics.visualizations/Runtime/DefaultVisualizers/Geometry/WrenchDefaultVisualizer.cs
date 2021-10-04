using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class WrenchDefaultVisualizer : DrawingVisualizer<WrenchMsg>
    {
        public float thickness = 0.01f;
        public float lengthScale = 1.0f;
        public float sphereRadius = 1.0f;
        public GameObject origin;
        [SerializeField]
        Color m_Color;

        public override void Draw(Drawing3d drawing, WrenchMsg message, MessageMetadata meta)
        {
            Draw<FLU>(message, drawing, SelectColor(m_Color, meta), origin.transform.position, lengthScale, sphereRadius, thickness);
        }

        public override Action CreateGUI(WrenchMsg message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };

        public static void Draw<C>(WrenchMsg message, Drawing3d drawing, Color color, Vector3 origin, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawArrow(origin, origin + message.force.From<C>() * lengthScale, color, thickness);
            VisualizationUtils.DrawAngularVelocityArrow(drawing, message.torque.From<C>(), origin, color, sphereRadius, thickness);
        }

    }
}
