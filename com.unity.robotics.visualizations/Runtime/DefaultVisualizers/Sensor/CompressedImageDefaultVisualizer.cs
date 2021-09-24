using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class CompressedImageDefaultVisualizer : TextureVisualizer<CompressedImageMsg>
{
    public override Texture2D CreateTexture(CompressedImageMsg message)
    {
        return message.ToTexture2D();
    }

    public override Action CreateGUI(CompressedImageMsg message, MessageMetadata meta, Texture2D tex)
    {
        return () =>
        {
            message.header.GUI();
            tex.GUITexture();
            GUILayout.Label($"Format: {message.format}");
        };
    }
}
