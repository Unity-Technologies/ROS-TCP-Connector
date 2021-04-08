using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerCompressedImage : BasicVisualizer<MCompressedImage>
{
    public Texture2D m_Tex { get; private set; }

    public override Action CreateGUI(MCompressedImage message, MessageMetadata meta, BasicDrawing drawing)
    {
        m_Tex = message.ToTexture2D();

        return () =>
		{
            message.header.GUI();
			message.GUI(m_Tex);
		};
    }
}
