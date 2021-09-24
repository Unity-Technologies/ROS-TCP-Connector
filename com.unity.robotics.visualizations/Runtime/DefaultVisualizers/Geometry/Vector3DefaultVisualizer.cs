using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class Vector3DefaultVisualizer : DrawingVisualizer<Vector3Msg>
    {
        [SerializeField]
        float m_Radius = 0.01f;
        [SerializeField]
        Color m_Color;
        [SerializeField]
        string m_Label;

        public override void Draw(Drawing3d drawing, Vector3Msg message, MessageMetadata meta)
        {
            Draw<FLU>(message, drawing, SelectColor(m_Color, meta), SelectLabel(m_Label, meta), m_Radius);
        }

        public static void Draw<C>(Vector3Msg message, Drawing3d drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 point = message.From<C>();
            drawing.DrawPoint(point, color, size);
            drawing.DrawLabel(label, point, color, size * 1.5f);
        }

        public static void Draw<C>(Vector3Msg message, Drawing3d drawing, Color color, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
        }

        public static void Draw<C>(Vector3Msg message, Drawing3d drawing, GameObject origin, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 point = message.From<C>();
            if (origin != null)
                point = origin.transform.TransformPoint(point);
            drawing.DrawPoint(point, color, size);
            drawing.DrawLabel(label, point, color, size * 1.5f);
        }


        public override Action CreateGUI(Vector3Msg message, MessageMetadata meta)
        {
            return () =>
            {
                message.GUI();
            };
        }
    }
}
