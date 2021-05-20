using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerRange : BasicVisualizer<MRange>
{
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, MRange message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta));
    }
        
    public override Action CreateGUI(MRange message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Radiation type: {(RangeRadiationTypes)message.radiation_type}\nFOV: {message.field_of_view} (rad)\nMin range: {message.min_range} (m)\nMax range: {message.max_range} (m)\nRange: {message.range} (m)");
    };
}
