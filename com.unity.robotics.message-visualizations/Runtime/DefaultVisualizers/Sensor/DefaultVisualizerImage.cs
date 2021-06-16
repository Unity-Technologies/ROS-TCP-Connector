using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerImage : TextureVisualFactory<MImage>
{
    bool m_ConvertBgr = true;
    bool m_FlipY = true;

    public override Texture2D CreateTexture(MImage message)
    {
        return message.data.Length > 0 ? message.ToTexture2D(m_ConvertBgr, m_FlipY) : null;
    }

    public override Action CreateGUI(MImage message, MessageMetadata meta, Texture2D tex)
    {
        return () =>
        {
            message.header.GUI();
            if (message.data.Length > 0)
            {
                GUILayout.BeginHorizontal();
                if (!message.encoding.Contains("1") && !message.encoding.Contains("mono") && !message.encoding.Contains("bayer")) m_ConvertBgr = GUILayout.Toggle(m_ConvertBgr, "From BGR");
                m_FlipY = GUILayout.Toggle(m_FlipY, "Flip Y");
                GUILayout.EndHorizontal();
            }
        
            tex.GUITexture();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nEncoding: {message.encoding}");
        };
    }
}
