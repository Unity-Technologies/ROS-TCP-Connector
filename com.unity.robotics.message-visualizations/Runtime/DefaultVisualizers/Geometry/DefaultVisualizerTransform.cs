using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTransform : DrawingVisualFactory<TransformMsg>
    {
        [SerializeField]
        float m_Size = 0.1f;
        [SerializeField]
        [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
        bool m_DrawUnityAxes;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        string m_Label;

        public override void Draw(BasicDrawing drawing, TransformMsg message, MessageMetadata meta)
        {
            message.Draw<FLU>(drawing, m_Size, m_DrawUnityAxes);
            drawing.DrawLabel(SelectLabel(m_Label, meta), message.translation.From<FLU>(), SelectColor(m_Color, meta), m_Size);
        }

        public override Action CreateGUI(TransformMsg message, MessageMetadata meta)
        {
            return () =>
            {
                message.GUI();
            };
        }
    }
}
