using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class CameraInfoDefaultVisualizer : TextureVisualizer<CameraInfoMsg>
{
    [SerializeField]
    public string ImageTopic;
    bool m_ViewK;
    bool m_ViewP;
    bool m_ViewR;

    public override Texture2D CreateTexture(CameraInfoMsg message)
    {
        // False if ROI not used, true if subwindow captured
        if (!message.roi.do_rectify)
            return null;

        ITextureVisual imageVisual = VisualizationUtils.GetVisual(ImageTopic) as ITextureVisual;
        if (imageVisual == null)
            return null;

        return message.roi.RegionOfInterestTexture(imageVisual.GetTexture());
    }

    public override Action CreateGUI(CameraInfoMsg message, MessageMetadata meta, Texture2D tex)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nDistortion model: {message.distortion_model}");
            GUILayout.Label($"Distortion parameters: {string.Join(", ", message.d)}");
            VisualizationUtils.GUIGrid(message.k, 3, "K", ref m_ViewK);
            VisualizationUtils.GUIGrid(message.r, 3, "R", ref m_ViewR);
            VisualizationUtils.GUIGrid(message.p, 3, "P", ref m_ViewP);
            GUILayout.Label($"Binning X: {message.binning_x}\nBinning Y: {message.binning_y}");
            message.roi.GUI(tex);
        };
    }
}
