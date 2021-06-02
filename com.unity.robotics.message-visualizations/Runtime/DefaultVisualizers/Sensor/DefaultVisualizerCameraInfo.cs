using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class DefaultVisualizerCameraInfo : BasicVisualizer<MCameraInfo>
{
    public enum AssociatedImageType { Image, CompressedImage };
    public AssociatedImageType m_ImageType;

    public override Action CreateGUI(MCameraInfo message, MessageMetadata meta, BasicDrawing drawing) 
    {
        return () => 
        {
            message.header.GUI();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nDistortion model: {message.distortion_model}");
            GUILayout.Label($"Distortion parameters: {String.Join(", ", message.D)}");
            MessageVisualizations.GUIGrid(message.K, 3, "K");
            MessageVisualizations.GUIGrid(message.R, 3, "R");
            MessageVisualizations.GUIGrid(message.P, 3, "P");
            GUILayout.Label($"Binning X: {message.binning_x}\nBinning Y: {message.binning_y}");
            message.roi.GUI(m_ImageType == AssociatedImageType.Image ? DefaultVisualizerImage.tex : DefaultVisualizerCompressedImage.m_Tex);
        };
    }
}
