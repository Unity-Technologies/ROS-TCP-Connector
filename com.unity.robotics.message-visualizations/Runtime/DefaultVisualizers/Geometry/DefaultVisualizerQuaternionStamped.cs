using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerQuaternionStamped : BasicVisualFactory<MQuaternionStamped>
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

        public override void Draw(BasicDrawing drawing, MQuaternionStamped message, MessageMetadata meta)
        {
            message.quaternion.Draw<FLU>(drawing, m_DrawAtPosition, m_Size, m_DrawUnityAxes);
            drawing.DrawLabel(SelectLabel(m_Label, meta), transform.position, SelectColor(m_Color, meta), m_Size);
        }

        public override Action CreateGUI(MQuaternionStamped message, MessageMetadata meta) => () =>
        {
            message.header.GUI();
            message.quaternion.GUI();
        };
    }
}