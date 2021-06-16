using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTwist : DrawingVisualFactory<MTwist>
    {
        public float m_Thickness = 0.01f;
        public float m_LengthScale = 1.0f;
        public float m_SphereRadius = 1.0f;
        public GameObject m_Origin;
        [SerializeField]
        Color m_Color;

        public override void Draw(BasicDrawing drawing, MTwist message, MessageMetadata meta)
        {
            message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Origin.transform.position, m_LengthScale, m_SphereRadius, m_Thickness);
        }

        public override Action CreateGUI(MTwist message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}