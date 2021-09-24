using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
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
            Vector3 prevPoint = Vector3.zero;
            Color color = Color.white;
            string label = "";

            MessageMetadata meta = messages.FirstOrDefault().Item2;
            color = VisualizationUtils.SelectColor(m_Color, meta);
            label = VisualizationUtils.SelectLabel(m_Label, meta);
            drawing.DrawPath(messages.Select(tuple => tuple.Item1.From<FLU>()), color, m_Thickness);
            drawing.DrawLabel(label, prevPoint, color);
        }
    }
}
