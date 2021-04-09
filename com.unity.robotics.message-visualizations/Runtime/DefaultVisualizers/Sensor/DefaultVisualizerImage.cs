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
            
            message.GUI(m_Tex);
            prevConvert = convertBgr;
            prevFlip = flipY;
		};
    }
}
