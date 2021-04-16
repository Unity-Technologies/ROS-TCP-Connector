using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerRegionOfInterest : BasicVisualizer<MRegionOfInterest>
{
    public Texture2D m_BaseImg;

    public override Action CreateGUI(MRegionOfInterest message, MessageMetadata meta, BasicDrawing drawing)
    {
        var roiTex = message.RegionOfInterestTexture(m_BaseImg, (int)message.height, (int)message.width);

        return () =>
        {
            message.GUI(roiTex);
        };
    }
}
