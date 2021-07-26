using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerRange : DrawingStampedVisualFactory<RangeMsg>
{
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, RangeMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingType(m_TFTrackingType, message.header);
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta));
    }

    public override Action CreateGUI(RangeMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Radiation type: {(Range_RadiationType_Constants)message.radiation_type}\nFOV: {message.field_of_view} (rad)\nMin range: {message.min_range} (m)\nMax range: {message.max_range} (m)\nRange: {message.range} (m)");
        };
    }
}
