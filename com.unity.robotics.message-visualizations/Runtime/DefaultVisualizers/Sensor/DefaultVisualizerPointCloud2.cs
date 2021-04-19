using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPointCloud2 : BasicVisualizer<MPointCloud2>
{
    public string[] m_XyzChannel = new string[] { "x", "y", "z" };
    public string m_RgbChannel = "ring";
    public float[] m_RgbRange = new float[] { 0, 31 };
    public string m_SizeChannel = "intensity";
    public float[] m_SizeRange = new float[] { 0, 100 };
    public float m_Size = 0.01f;
    public bool m_UseRgbChannel = false;
    public bool m_UseSizeChannel = false;
    
    public override void Draw(BasicDrawing drawing, MPointCloud2 message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color, m_XyzChannel, m_RgbChannel, m_UseRgbChannel ? m_RgbRange : null, m_SizeChannel, m_UseSizeChannel ? m_SizeRange : null, m_Size);
    }

    public override Action CreateGUI(MPointCloud2 message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Height x Width: {message.height}x{message.width}\nData length: {message.data.Length}\nPoint step: {message.point_step}\nRow step: {message.row_step}\nIs dense: {message.is_dense}");
        foreach (MPointField field in message.fields)
        {
            field.GUI();
        }
    };
}