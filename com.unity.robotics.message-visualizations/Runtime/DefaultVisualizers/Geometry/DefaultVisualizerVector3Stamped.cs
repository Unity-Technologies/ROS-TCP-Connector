using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerVector3Stamped : DrawingStampedVisualFactory<Vector3StampedMsg>
    {
        [SerializeField]
        float m_Radius = 0.01f;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        string m_Label;

        public override void Draw(BasicDrawing drawing, Vector3StampedMsg message, MessageMetadata meta)
        {
            drawing.SetTFTrackingType(m_TFTrackingType, message.header);
            message.vector.Draw<FLU>(drawing, SelectColor(m_Color, meta), SelectLabel(m_Label, meta), m_Radius);
        }

        public override Action CreateGUI(Vector3StampedMsg message, MessageMetadata meta)
        {
            return () =>
            {
                message.header.GUI();
                message.vector.GUI();
            };
        }
    }
}
