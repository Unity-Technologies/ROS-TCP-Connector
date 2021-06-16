using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCompressedImage : TextureVisualFactory<MCompressedImage>
{
    public override Texture2D CreateTexture(MCompressedImage message)
    {
        return message.ToTexture2D();
    }

    public override Action CreateGUI(MCompressedImage message, MessageMetadata meta, Texture2D tex) => () =>
    {
        message.header.GUI();
        tex.GUITexture();
        GUILayout.Label($"Format: {message.format}");
    };
}
