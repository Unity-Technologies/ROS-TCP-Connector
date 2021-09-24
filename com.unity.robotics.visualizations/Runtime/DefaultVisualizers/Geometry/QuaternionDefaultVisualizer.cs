using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class QuaternionDefaultVisualizer : DrawingVisualizer<QuaternionMsg>
    {
        [SerializeField]
        float m_Size = 0.1f;
        [SerializeField]
        GameObject m_DrawAtPosition;
        [SerializeField]
        [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
        bool m_DrawUnityAxes;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        string m_Label;

        public override void Draw(Drawing3d drawing, QuaternionMsg message, MessageMetadata meta)
        {
            Draw<FLU>(message, drawing, m_DrawAtPosition, m_Size, m_DrawUnityAxes);
            drawing.DrawLabel(SelectLabel(m_Label, meta), transform.position, SelectColor(m_Color, meta), m_Size);
        }

        public static void Draw<C>(QuaternionMsg message, Drawing3d drawing, GameObject drawAtPosition = null, float size = 0.1f, bool drawUnityAxes = false)
where C : ICoordinateSpace, new()
        {
            Vector3 position = drawAtPosition != null ? drawAtPosition.transform.position : Vector3.zero;
            VisualizationUtils.DrawAxisVectors<C>(drawing, position.To<C>(), message, size, drawUnityAxes);
        }

        public static void Draw<C>(QuaternionMsg message, Drawing3d drawing, Vector3 position, float size = 0.1f, bool drawUnityAxes = false)
            where C : ICoordinateSpace, new()
        {
            VisualizationUtils.DrawAxisVectors<C>(drawing, position.To<C>(), message, size, drawUnityAxes);
        }

        public override Action CreateGUI(QuaternionMsg message, MessageMetadata meta)
        {
            return () =>
            {
                message.GUI();
            };
        }
    }
}
