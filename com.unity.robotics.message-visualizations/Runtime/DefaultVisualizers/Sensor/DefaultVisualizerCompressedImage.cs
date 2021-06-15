using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCompressedImage : BasicTextureVisualFactory<MCompressedImage>
{
    protected override Texture2D CreateTexture(MCompressedImage message)
    {
        return message.ToTexture2D();
    }

    public override Action CreateGUI(MCompressedImage message, MessageMetadata meta, Texture2D tex) => () =>
    {
        message.header.GUI();
        // TODO: Rescale/recenter image based on window height/width
        var origRatio = tex.width / (float)tex.height;
        GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), tex);
        GUILayout.Label($"Format: {message.format}");
    };
}
