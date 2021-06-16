using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCameraInfo : TextureVisualFactory<MCameraInfo>
{
    public string imageTopic;
    bool m_ViewK;
    bool m_ViewR;
    bool m_ViewP;

    public override Texture2D CreateTexture(MCameraInfo message)
    {
        // False if ROI not used, true if subwindow captured
        if (message.roi.do_rectify)
        {
            var imageState = ROSConnection.instance.HUDPanel.GetVisualizationState(imageTopic, true);
            if (imageState.WindowContents != null)
            {
                var imageVisual = imageState.WindowContents.GetVisual() as ITextureVisual;
                if (imageVisual != null)
                {
                    return message.roi.RegionOfInterestTexture(imageVisual.GetTexture());
                }       
            }
        }
        return null;
    }

    public override Action CreateGUI(MCameraInfo message, MessageMetadata meta, Texture2D tex) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Height x Width: {message.height}x{message.width}\nDistortion model: {message.distortion_model}");
        GUILayout.Label($"Distortion parameters: {string.Join(", ", message.D)}");
        MessageVisualizations.GUIGrid(message.K, 3, "K", ref m_ViewK);
        MessageVisualizations.GUIGrid(message.R, 3, "R", ref m_ViewR);
        MessageVisualizations.GUIGrid(message.P, 3, "P", ref m_ViewP);
        GUILayout.Label($"Binning X: {message.binning_x}\nBinning Y: {message.binning_y}");
        message.roi.GUI(tex);
    };
}
