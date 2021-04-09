using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerRange : BasicVisualizer<MRange>
{
    bool m_ShowMax = false;
    bool m_ShowMin = false;

    public override void Draw(BasicDrawing drawing, MRange message, MessageMetadata meta, Color color, string label)
    {
        var s = Mathf.Asin(message.field_of_view);
        var c = Mathf.Acos(message.field_of_view);
        if (m_ShowMax)
            drawing.DrawCone(new Vector3(message.max_range * c, 0, message.max_range * s), Vector3.zero, Color.red, Mathf.Rad2Deg * message.field_of_view / 2);
        drawing.DrawCone(new Vector3(message.range * c, 0, message.range * s), Vector3.zero, Color.yellow, Mathf.Rad2Deg * message.field_of_view / 2);
        if (m_ShowMin)
            drawing.DrawCone(new Vector3(message.min_range * c, 0, message.min_range * s), Vector3.zero, Color.blue, Mathf.Rad2Deg * message.field_of_view / 2);
    }
        
    public override Action CreateGUI(MRange message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        GUILayout.BeginHorizontal();
        m_ShowMax = GUILayout.Toggle(m_ShowMax, "Show max");
        m_ShowMin = GUILayout.Toggle(m_ShowMin, "Show min");
        GUILayout.EndHorizontal();
        message.header.GUI();
        message.GUI();
    };
}
