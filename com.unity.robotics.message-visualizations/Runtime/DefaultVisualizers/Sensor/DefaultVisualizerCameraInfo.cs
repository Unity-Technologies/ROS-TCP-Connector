using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCameraInfo : BasicVisualizer<MCameraInfo>
{
    public enum AssociatedImageType { Image, CompressedImage };
    public AssociatedImageType m_ImageType;
    static Texture2D m_Tex;

    public override Action CreateGUI(MCameraInfo message, MessageMetadata meta, BasicDrawing drawing) 
    {
        // False if ROI not used, true if subwindow captured
        if (message.roi.do_rectify) 
        {
            m_Tex = message.roi.RegionOfInterestTexture(m_ImageType == AssociatedImageType.Image ? DefaultVisualizerImage.m_Tex : DefaultVisualizerCompressedImage.m_Tex);
        }
        
        return () => 
        {
            message.header.GUI();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nDistortion model: {message.distortion_model}");
            GUILayout.Label($"Distortion parameters: {String.Join(", ", message.D)}");
            MessageVisualizations.GUIGrid(message.K, 3, "K");
            MessageVisualizations.GUIGrid(message.R, 3, "R");
            MessageVisualizations.GUIGrid(message.P, 3, "P");
            GUILayout.Label($"Binning X: {message.binning_x}\nBinning Y: {message.binning_y}");
            message.roi.GUI(m_Tex);
        };
    }
}
