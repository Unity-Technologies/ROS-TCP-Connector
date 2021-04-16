using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerImage : BasicVisualizer<MImage>
{
    public Texture2D m_Tex { get; private set; }

    public override Action CreateGUI(MImage message, MessageMetadata meta, BasicDrawing drawing)
    {
        bool convertBgr = true;
        bool prevConvert = true;
        bool flipY = true;
        bool prevFlip = true;
        m_Tex = message.ToTexture2D(convertBgr, flipY);

		return () =>
		{
            message.header.GUI();
            GUILayout.BeginHorizontal();
            if (!message.encoding.Contains("1") && !message.encoding.Contains("mono") && !message.encoding.Contains("bayer"))
                convertBgr = GUILayout.Toggle(convertBgr, "From BGR");
            flipY = GUILayout.Toggle(flipY, "Flip Y");
            GUILayout.EndHorizontal();

            if (convertBgr != prevConvert || flipY != prevFlip)
                m_Tex = message.ToTexture2D(convertBgr, flipY);
            
            // TODO: Rescale/recenter image based on window height/width
            var origRatio = (float)m_Tex.width / (float)m_Tex.height;
            UnityEngine.GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), m_Tex);
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nEncoding: {message.encoding}");

            prevConvert = convertBgr;
            prevFlip = flipY;
		};
    }
}
