using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class TransformDefaultVisualizer : DrawingVisualizer<TransformMsg>
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

        public override void Draw(Drawing3d drawing, TransformMsg message, MessageMetadata meta)
        {
            Draw<FLU>(message, drawing, m_Size, m_DrawUnityAxes);
            drawing.DrawLabel(SelectLabel(m_Label, meta), message.translation.From<FLU>(), SelectColor(m_Color, meta), m_Size);
        }

        public static void Draw<C>(TransformMsg transform, Drawing3d drawing, float size = 0.01f, bool drawUnityAxes = false) where C : ICoordinateSpace, new()
        {
            QuaternionDefaultVisualizer.Draw<C>(transform.rotation, drawing, transform.translation.From<C>(), size, drawUnityAxes);
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
