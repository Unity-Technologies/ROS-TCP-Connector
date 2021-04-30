using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerTransformStamped : BasicVisualizer<MTransformStamped>
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

        public override void Draw(BasicDrawing drawing, MTransformStamped message, MessageMetadata meta)
        {
            message.transform.Draw<FLU>(drawing, m_Size, m_DrawUnityAxes);
            drawing.DrawLabel(SelectLabel(m_Label, meta), message.transform.translation.From<FLU>(), SelectColor(m_Color, meta), m_Size);
        }

        public override System.Action CreateGUI(MTransformStamped message, MessageMetadata meta, BasicDrawing drawing) => () =>
        {
            message.header.GUI();
            message.transform.GUI();
        };
    }
}
