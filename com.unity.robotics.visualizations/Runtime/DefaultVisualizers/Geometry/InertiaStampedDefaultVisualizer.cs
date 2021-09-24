using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class InertiaStampedDefaultVisualizer : DrawingVisualizer<InertiaStampedMsg>
{
    public GameObject m_Origin;
    public float m_Radius;
    [SerializeField]
    Color m_Color;
    [SerializeField]
    string m_Label = "Center of mass";
    [SerializeField]
    TFTrackingSettings m_TFTrackingType;

    public override void Draw(Drawing3d drawing, InertiaStampedMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingType, message.header);
        Vector3DefaultVisualizer.Draw<FLU>(message.inertia.com, drawing, m_Origin, SelectColor(m_Color, meta), SelectLabel(m_Label, meta), m_Radius);
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
