using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCompressedImage : BasicVisualizer<MCompressedImage>
{
    public static Texture2D m_Tex { get; private set; }

    public override Action CreateGUI(MCompressedImage message, MessageMetadata meta, BasicDrawing drawing)
    {
        m_Tex = message.ToTexture2D();

        return () =>
		{
            message.header.GUI();
			// TODO: Rescale/recenter image based on window height/width
            var origRatio = (float)m_Tex.width / (float)m_Tex.height;
            UnityEngine.GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), m_Tex);
            GUILayout.Label($"Format: {message.format}");
		};
    }
}
