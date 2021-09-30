using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class TransformStampedDefaultVisualizer : DrawingVisualizer<TransformStampedMsg>
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
        [SerializeField]
        TFTrackingSettings m_TFTrackingSettings;

        public override void Draw(Drawing3d drawing, TransformStampedMsg message, MessageMetadata meta)
        {
            drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
            TransformDefaultVisualizer.Draw<FLU>(message.transform, drawing, m_Size, m_DrawUnityAxes);
            drawing.DrawLabel(SelectLabel(m_Label, meta), message.transform.translation.From<FLU>(), SelectColor(m_Color, meta), m_Size);
        }

        public override Action CreateGUI(TransformStampedMsg message, MessageMetadata meta)
        {
            return () =>
            {
                message.header.GUI();
                message.transform.GUI();
            };
        }
    }
}
