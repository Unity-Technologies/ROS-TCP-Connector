using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerImage : BasicVisualizer<MImage>, ITextureMessageVisualization
{
    public Message message { get; }
    public MessageMetadata meta { get; }
    public bool hasDrawing 
    { 
        get { return false; } 
        set { hasDrawing = value; } }
    public bool hasAction { get; set; }

    Texture2D m_Tex;
    bool m_ConvertBgr = true;
    bool m_PrevConvert = true;
    bool m_FlipY = true;
    bool m_PrevFlip = true;
    public void Delete() {}
    public void OnGUI() {}
    public Texture2D GetTexture()
    {
        return m_Tex;
    }

    public override Action CreateGUI(MImage message, MessageMetadata meta, BasicDrawing drawing)
    {
        if (message.data.Length > 0)
        {
            m_Tex = message.ToTexture2D(m_ConvertBgr, m_FlipY);
        }

		return () =>
		{
            message.header.GUI();
            if (message.data.Length > 0)
            {
                GUILayout.BeginHorizontal();
                if (!message.encoding.Contains("1") && !message.encoding.Contains("mono") && !message.encoding.Contains("bayer"))
                {
                    m_ConvertBgr = GUILayout.Toggle(m_ConvertBgr, "From BGR");
                }
                m_FlipY = GUILayout.Toggle(m_FlipY, "Flip Y");
                GUILayout.EndHorizontal();

                if (m_ConvertBgr != m_PrevConvert || m_FlipY != m_PrevFlip)
                {
                    m_Tex = message.ToTexture2D(m_ConvertBgr, m_FlipY);
                }
            }

            // TODO: Rescale/recenter image based on window height/width
            if (m_Tex != null)
            {
                var origRatio = (float)m_Tex.width / (float)m_Tex.height;
                UnityEngine.GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), m_Tex);
            }
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nEncoding: {message.encoding}");

            m_PrevConvert = m_ConvertBgr;
            m_PrevFlip = m_FlipY;
		};
    }
}
