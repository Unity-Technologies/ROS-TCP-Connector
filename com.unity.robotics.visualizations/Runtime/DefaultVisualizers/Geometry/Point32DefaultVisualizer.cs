using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class Point32DefaultVisualizer : DrawingVisualizer<Point32Msg>
    {
        [SerializeField]
        float m_Radius = 0.01f;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        string m_Label;

        public override void Draw(Drawing3d drawing, Point32Msg message, MessageMetadata meta)
        {
            Draw<FLU>(message, drawing, SelectColor(m_Color, meta), SelectLabel(m_Label, meta), m_Radius);
        }

        public static void Draw<C>(Point32Msg message, Drawing3d drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void Draw<C>(Point32Msg message, Drawing3d drawing, Color color, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
        }

        public override Action CreateGUI(Point32Msg message, MessageMetadata meta)
        {
            return () =>
            {
                message.GUI();
            };
        }
    }
}
