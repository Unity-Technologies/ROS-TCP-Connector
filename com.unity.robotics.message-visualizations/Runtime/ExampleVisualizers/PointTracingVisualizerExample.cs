using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class PointTracingVisualizerExample : HistoryDrawingVisualizer<PointMsg>
    {
        [SerializeField]
        Color m_Color;
        [SerializeField]
        float m_Thickness = 0.1f;
        [SerializeField]
        string m_Label;

        public override void Draw(Drawing3d drawing, IEnumerable<Tuple<PointMsg, MessageMetadata>> messages)
        {
            bool firstPass = true;
            Vector3 prevPoint = Vector3.zero;
            Color color = Color.white;
            string label = "";

            foreach ((PointMsg rosPoint, MessageMetadata meta) in messages)
            {
                Vector3 point = rosPoint.From<FLU>();
                if (firstPass)
                {
                    color = MessageVisualizationUtils.SelectColor(m_Color, meta);
                    label = MessageVisualizationUtils.SelectLabel(m_Label, meta);
                    firstPass = false;
                }
                else
                {
                    drawing.DrawLine(prevPoint, point, color, m_Thickness);
                }
                prevPoint = point;
            }

            drawing.DrawLabel(label, prevPoint, color);
        }
    }
}
