using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTransform : BasicVisualizer<MTransform>
    {
        [SerializeField]
        float m_Size = 0.1f;
        [SerializeField]
        [Tooltip("If ticked, draw the axis lines for Unity coordinates. Otherwise, draw the axis lines for ROS coordinates (FLU).")]
        bool m_DrawUnityAxes;

        public override void Draw(BasicDrawing drawing, MTransform message, MessageMetadata meta, Color color, string label)
        {
            message.Draw<FLU>(drawing, m_Size, m_DrawUnityAxes);
            drawing.DrawLabel(label, message.translation.From<FLU>(), color, m_Size);
        }

        public override System.Action CreateGUI(MTransform message, MessageMetadata meta, BasicDrawing drawing) => () =>
        {
            message.GUI();
        };
    }
}
