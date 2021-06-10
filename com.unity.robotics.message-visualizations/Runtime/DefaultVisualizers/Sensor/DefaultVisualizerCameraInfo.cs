using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCameraInfo : BasicVisualizer<MCameraInfo>
{
    public string imageTopic;
    Texture2D m_Tex;
    ITextureMessageVisualization m_TextureVisualization;
    IVisualizer m_Visualizer;

    public override Action CreateGUI(MCameraInfo message, MessageMetadata meta, BasicDrawing drawing)
    {
        // False if ROI not used, true if subwindow captured
        if (message.roi.do_rectify)
        {
            m_Visualizer = ROSConnection.instance.HudPanel.GetVisualizer(imageTopic);
            m_TextureVisualization = m_Visualizer as ITextureMessageVisualization;
            if (m_TextureVisualization != null)
            {
                m_Tex = message.roi.RegionOfInterestTexture(m_TextureVisualization.GetTexture());
            }
            else
            {
                Debug.LogError($"{imageTopic} was not a texture visualizer!");
                return null;
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
            message.roi.GUI(m_Tex);
        };
    }
}
