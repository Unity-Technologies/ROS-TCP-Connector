using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerImage : BasicTextureVisualizer<MImage>
{
    bool m_ConvertBgr = true;
    bool m_FlipY = true;
    bool m_PrevConvert = true;
    bool m_PrevFlip = true;

    public override Action CreateGUI(MImage message, MessageMetadata meta, BasicDrawing drawing)
    {
        if (message.data.Length > 0)
        {
            if (m_Visualization == null) m_Visualization = (ITextureMessageVisualization)VisualizationRegistry.GetVisualization(meta.Topic);
            m_Visualization.SetTexture(message.ToTexture2D(m_ConvertBgr, m_FlipY));
        }

        return () =>
        {
            message.header.GUI();
            if (message.data.Length > 0)
            {
                GUILayout.BeginHorizontal();
                if (!message.encoding.Contains("1") && !message.encoding.Contains("mono") && !message.encoding.Contains("bayer")) m_ConvertBgr = GUILayout.Toggle(m_ConvertBgr, "From BGR");
                m_FlipY = GUILayout.Toggle(m_FlipY, "Flip Y");
                GUILayout.EndHorizontal();

                if (m_ConvertBgr != m_PrevConvert || m_FlipY != m_PrevFlip) m_Visualization.SetTexture(message.ToTexture2D(m_ConvertBgr, m_FlipY));
            }

            // TODO: Rescale/recenter image based on window height/width
            if (m_Visualization.GetTexture() != null)
            {
                var origRatio = m_Visualization.GetTexture().width / (float)m_Visualization.GetTexture().height;
                GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), m_Visualization.GetTexture());
            }

            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nEncoding: {message.encoding}");

            m_PrevConvert = m_ConvertBgr;
            m_PrevFlip = m_FlipY;
        };
    }
}
