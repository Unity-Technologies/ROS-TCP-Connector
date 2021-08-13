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
    [SerializeField]
    TFTrackingType m_TFTrackingType;

    public override void Draw(BasicDrawing drawing, InertiaStampedMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        DefaultVisualizerVector3.Draw<FLU>(message.inertia.com, drawing, m_Origin, SelectColor(m_Color, meta), SelectLabel(m_Label, meta), m_Radius);
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
