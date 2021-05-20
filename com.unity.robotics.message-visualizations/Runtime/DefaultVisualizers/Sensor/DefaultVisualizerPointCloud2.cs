using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public enum ColorMode
{
    HSV, RGB
}

public class DefaultVisualizerPointCloud2 : BasicVisualizer<MPointCloud2>
{
    public PointCloud2VisualizerSettings m_Settings;
    [SerializeField]
    Color m_Color;

    public override void Start()
    {
        if (m_Settings == null)
        {
            Debug.Log($"Visualizer settings were null!");
        }
        base.Start();
    }
    
    public override void Draw(BasicDrawing drawing, MPointCloud2 message, MessageMetadata meta)
    {
        if (m_Settings.channels == null)
            m_Settings.channels = message.fields;
        
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta), m_Settings);
    }

    public override Action CreateGUI(MPointCloud2 message, MessageMetadata meta, BasicDrawing drawing)
    {
        var formatDict = new Dictionary<PointFieldFormat, List<string>>();

        foreach (MPointField field in message.fields)
        {
            if (formatDict.ContainsKey((PointFieldFormat)field.datatype))
                formatDict[(PointFieldFormat)field.datatype].Add(field.name);
            else 
                formatDict.Add((PointFieldFormat)field.datatype, new List<string>() { field.name });
        }

        string formats = "";
        foreach (var f in formatDict)
        {
            if (f.Value.Count > 0)
                formats += $"{f.Key}: {String.Join(", ", f.Value)}\n";
        }

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nData length: {message.data.Length}\nPoint step: {message.point_step}\nRow step: {message.row_step}\nIs dense: {message.is_dense}");
            GUILayout.Label($"Channels:\n{formats}");
        };
    }
}