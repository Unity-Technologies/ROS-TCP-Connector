using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCameraInfo : BasicTextureVisualizer<MCameraInfo>
{
    public string imageTopic;
    ITextureMessageVisualization m_ImageVisualization;
    ITextureMessageVisualization m_TextureVisualization;

    public override Action CreateGUI(MCameraInfo message, MessageMetadata meta, BasicDrawing drawing)
    {
        // False if ROI not used, true if subwindow captured
        if (message.roi.do_rectify)
        {
            m_ImageVisualization = ROSConnection.instance.HudPanel.GetVisualization(imageTopic) as ITextureMessageVisualization;
            if (m_ImageVisualization != null)
            {
                m_TextureVisualization ??= (ITextureMessageVisualization)VisualizationRegistry.GetVisualization(meta.Topic);
                // if (m_TextureVisualization != null)
                // {
                    m_TextureVisualization?.Delete();
                    m_TextureVisualization?.SetTexture(message.roi.RegionOfInterestTexture(m_ImageVisualization.GetTexture()));
                // }
            }
        }

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nDistortion model: {message.distortion_model}");
            GUILayout.Label($"Distortion parameters: {string.Join(", ", message.D)}");
            MessageVisualizations.GUIGrid(message.K, 3, "K");
            MessageVisualizations.GUIGrid(message.R, 3, "R");
            MessageVisualizations.GUIGrid(message.P, 3, "P");
            GUILayout.Label($"Binning X: {message.binning_x}\nBinning Y: {message.binning_y}");
            message.roi.GUI(m_TextureVisualization?.GetTexture());
        };
    }
}
