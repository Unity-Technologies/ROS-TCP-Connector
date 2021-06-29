using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public enum ColorMode
{
    HSV,
    RGB
}

public class DefaultVisualizerPointCloud2 : DrawingVisualFactory<PointCloud2Msg>
{
    public PointCloud2VisualizerSettings m_Settings;
    [SerializeField]
    Color m_Color;

    public override void Start()
    {
        if (m_Settings == null) Debug.Log("Visualizer settings were null!");
        base.Start();
    }

    public override void Draw(BasicDrawing drawing, PointCloud2Msg message, MessageMetadata meta)
    {
        if (m_Settings.channels == null)
            m_Settings.channels = message.fields;

        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Settings);
    }

    public override Action CreateGUI(PointCloud2Msg message, MessageMetadata meta)
    {
        var formatDict = new Dictionary<PointField_Format_Constants, List<string>>();

        foreach (var field in message.fields)
            if (formatDict.ContainsKey((PointField_Format_Constants)field.datatype))
                formatDict[(PointField_Format_Constants)field.datatype].Add(field.name);
            else
                formatDict.Add((PointField_Format_Constants)field.datatype, new List<string> { field.name });

        var formats = "";
        foreach (var f in formatDict)
            if (f.Value.Count > 0)
                formats += $"{f.Key}: {string.Join(", ", f.Value)}\n";

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nData length: {message.data.Length}\nPoint step: {message.point_step}\nRow step: {message.row_step}\nIs dense: {message.is_dense}");
            GUILayout.Label($"Channels:\n{formats}");
        };
    }
}
