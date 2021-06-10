using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCompressedImage : BasicVisualizer<MCompressedImage>, ITextureMessageVisualization
{
    Texture2D m_Tex;
    public Message message { get; }
    public MessageMetadata meta { get; }

    public bool hasDrawing
    {
        get => false;
        set => hasDrawing = value;
    }

    public bool hasAction { get; set; }
    public void Delete() { }
    public void OnGUI() { }

    public Texture2D GetTexture()
    {
        return m_Tex;
    }

    public override Action CreateGUI(MCompressedImage message, MessageMetadata meta, BasicDrawing drawing)
    {
        m_Tex = message.ToTexture2D();

        return () =>
        {
            message.header.GUI();

            // TODO: Rescale/recenter image based on window height/width
            var origRatio = m_Tex.width / (float)m_Tex.height;
            GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), m_Tex);
            GUILayout.Label($"Format: {message.format}");
        };
    }
}
