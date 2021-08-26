using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class ImageDefaultVisualizer : TextureVisualizer<ImageMsg>
{
    bool m_ConvertBgr = true;
    bool m_Debayer = true;
    bool m_FlipY = true;

    public override Texture2D CreateTexture(ImageMsg message)
    {
        return message.data.Length > 0 ? message.ToTexture2D(m_ConvertBgr, m_Debayer, m_FlipY) : null;
    }

    public override Action CreateGUI(ImageMsg message, MessageMetadata meta, Texture2D tex)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"{message.height}x{message.width}, encoding: {message.encoding}");
            if (message.data.Length > 0)
            {
                GUILayout.BeginHorizontal();
                if (message.EncodingRequiresBGRConversion())
                    m_ConvertBgr = GUILayout.Toggle(m_ConvertBgr, "From BGR");
                if (message.EncodingRequiresBGRConversion())
                    m_Debayer = GUILayout.Toggle(m_Debayer, "Debayer");
                m_FlipY = GUILayout.Toggle(m_FlipY, "Flip Y");
                GUILayout.EndHorizontal();
            }

            tex.GUITexture();
        };
    }
}
