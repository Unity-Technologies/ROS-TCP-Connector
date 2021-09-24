using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class PolygonDefaultVisualizer : DrawingVisualizer<PolygonMsg>
    {
        [SerializeField]
        float m_Thickness = 0.1f;
        [SerializeField]
        Color m_Color;

        public override void Draw(Drawing3d drawing, PolygonMsg message, MessageMetadata meta)
        {
            Draw<FLU>(message, drawing, SelectColor(m_Color, meta), m_Thickness);
        }

        public static void Draw<C>(PolygonMsg message, Drawing3d drawing, Color color, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 prevPos = message.points[message.points.Length - 1].From<C>();
            foreach (Point32Msg p in message.points)
            {
                Vector3 curPos = p.From<C>();
                drawing.DrawLine(prevPos, curPos, color, thickness);
                prevPos = curPos;
            }
        }

        public override Action CreateGUI(PolygonMsg message, MessageMetadata meta)
        {
            return () =>
            {
                message.GUI();
            };
        }
    }
}
