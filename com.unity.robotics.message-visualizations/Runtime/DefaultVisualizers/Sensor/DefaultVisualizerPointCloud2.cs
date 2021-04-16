using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerPointCloud2 : BasicVisualizer<MPointCloud2>
{
    public override void Draw(BasicDrawing drawing, MPointCloud2 message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color);
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
