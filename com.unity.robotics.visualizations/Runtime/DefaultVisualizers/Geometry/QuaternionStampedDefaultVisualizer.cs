using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class QuaternionStampedDefaultVisualizer : DrawingVisualizer<QuaternionStampedMsg>
    {
        [SerializeField]
        float m_Size = 0.01f;
        [SerializeField]
        GameObject m_DrawAtPosition;
        [SerializeField]
        [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
        bool m_DrawUnityAxes;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        string m_Label;
        [SerializeField]
        TFTrackingSettings m_TFTrackingSettings;

        public override void Draw(Drawing3d drawing, QuaternionStampedMsg message, MessageMetadata meta)
        {
            drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
            QuaternionDefaultVisualizer.Draw<FLU>(message.quaternion, drawing, m_DrawAtPosition, m_Size, m_DrawUnityAxes);
            drawing.DrawLabel(SelectLabel(m_Label, meta), transform.position, SelectColor(m_Color, meta), m_Size);
        }

        public override Action CreateGUI(QuaternionStampedMsg message, MessageMetadata meta)
        {
            return () =>
            {
                message.header.GUI();
                message.quaternion.GUI();
            };
        }
    }
}
