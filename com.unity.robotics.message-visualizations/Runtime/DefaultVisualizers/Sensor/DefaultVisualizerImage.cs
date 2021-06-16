using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerImage : BasicTextureVisualFactory<MImage>
{
    bool m_ConvertBgr = true;
    bool m_FlipY = true;
    bool m_PrevConvert = true;
    bool m_PrevFlip = true;

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
        
                // TODO:
                // if (m_ConvertBgr != m_PrevConvert || m_FlipY != m_PrevFlip)
                // {
                //     // m_Visualization.SetTexture(message.ToTexture2D(m_ConvertBgr, m_FlipY));
                // }
            }
        
            // TODO: Rescale/recenter image based on window height/width
            if (tex != null)
            {
                var origRatio = tex.width / (float)tex.height;
                GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), tex);
            }
        
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nEncoding: {message.encoding}");
        
            m_PrevConvert = m_ConvertBgr;
            m_PrevFlip = m_FlipY;
        };
    }
}
