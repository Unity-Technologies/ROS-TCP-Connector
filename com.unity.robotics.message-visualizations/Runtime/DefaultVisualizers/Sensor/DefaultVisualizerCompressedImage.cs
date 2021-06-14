using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCompressedImage : BasicTextureVisualizer<MCompressedImage>
{
    public override Action CreateGUI(MCompressedImage message, MessageMetadata meta, BasicDrawing drawing)
    {
        m_Visualization ??= (ITextureMessageVisualization)VisualizationRegistry.GetVisualization(meta.Topic);
        if (m_Visualization != null)
        {
            m_Visualization.Delete();
            m_Visualization.SetTexture(message.ToTexture2D());
        }

        return () =>
        {
            message.header.GUI();

            // TODO: Rescale/recenter image based on window height/width
            if (m_Visualization?.GetTexture() != null)
            {
                var origRatio = m_Visualization.GetTexture().width / (float)m_Visualization.GetTexture().height;
                GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), m_Visualization?.GetTexture());
                GUILayout.Label($"Format: {message.format}");
            }
        };
    }
}
