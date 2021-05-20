using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerCameraInfo : BasicVisualizer<MCameraInfo>
{
    public Texture2D m_BaseImg;

    public override Action CreateGUI(MCameraInfo message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Height x Width: {message.height}x{message.width}\nDistortion model: {message.distortion_model}");
        GUILayout.Label($"Distortion parameters: {String.Join(", ", message.D)}");
        MessageVisualizations.GUIGrid(message.K, 3, "K");
        MessageVisualizations.GUIGrid(message.R, 3, "R");
        MessageVisualizations.GUIGrid(message.P, 3, "P");
        GUILayout.Label($"Binning X: {message.binning_x}\nBinning Y: {message.binning_y}");
        if (m_BaseImg != null)
            message.roi.GUI(m_BaseImg);
    };
}
