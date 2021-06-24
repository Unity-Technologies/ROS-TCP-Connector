using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTwistStamped : DrawingVisualFactory<MTwistStamped>
    {
        public float thickness = 0.01f;
        public float lengthScale = 1.0f;
        public float sphereRadius = 1.0f;
        public GameObject origin;
        [SerializeField]
        Color m_Color;

        public override void Draw(BasicDrawing drawing, MTwistStamped message, MessageMetadata meta)
        {
            message.twist.Draw<FLU>(drawing, SelectColor(m_Color, meta), origin.transform.position, lengthScale, sphereRadius, thickness);
        }

        public override Action CreateGUI(MTwistStamped message, MessageMetadata meta) => () =>
        {
            message.header.GUI();
            message.twist.GUI();
        };
    }
}
