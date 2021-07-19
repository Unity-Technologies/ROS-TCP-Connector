using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerInertiaStamped : DrawingVisualFactory<InertiaStampedMsg>
{
    public GameObject m_Origin;
    public float m_Radius;
    [SerializeField]
    Color m_Color;
    [SerializeField]
    string m_Label = "Center of mass";

    public override void Draw(BasicDrawing drawing, InertiaStampedMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        message.inertia.com.Draw<FLU>(drawing, m_Origin, SelectColor(m_Color, meta), SelectLabel(m_Label, meta), m_Radius);
    }

    public override Action CreateGUI(InertiaStampedMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.inertia.GUI();
        };
    }
}
