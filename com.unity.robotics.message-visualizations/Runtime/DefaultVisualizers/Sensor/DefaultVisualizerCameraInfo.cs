using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class DefaultVisualizerCameraInfo : BasicVisualizer<MCameraInfo>
{
    public string m_ImageTopic;
    IVisualizer m_Visualizer;
    HUDVisualizationRule m_Rule;
    static Texture2D m_Tex;

    public override Action CreateGUI(MCameraInfo message, MessageMetadata meta, BasicDrawing drawing) 
    {
        // False if ROI not used, true if subwindow captured
        if (message.roi.do_rectify) 
        {
            m_Visualizer = ROSConnection.instance.HudPanel.GetVisualizer(m_ImageTopic);
            if (m_Visualizer != null)
            {
                m_Rule = ROSConnection.instance.HudPanel.AllTopics[m_ImageTopic];
                if (m_Rule.RosMessageName == "sensor_msgs/CompressedImage")
                {
                    m_Tex = message.roi.RegionOfInterestTexture(((DefaultVisualizerCompressedImage)m_Visualizer).m_Tex);
                }
                else if (m_Rule.RosMessageName == "sensor_msgs/Image")
                {
                    m_Tex = message.roi.RegionOfInterestTexture(((DefaultVisualizerImage)m_Visualizer).m_Tex);
                }
                else
                {
                    Debug.LogError($"Message type {m_Rule.RosMessageName} is not supported with CameraInfo!");
                }
            }
        }
        
        return () => 
        {
            message.header.GUI();
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nDistortion model: {message.distortion_model}");
            GUILayout.Label($"Distortion parameters: {String.Join(", ", message.D)}");
            MessageVisualizations.GUIGrid(message.K, 3, "K");
            MessageVisualizations.GUIGrid(message.R, 3, "R");
            MessageVisualizations.GUIGrid(message.P, 3, "P");
            GUILayout.Label($"Binning X: {message.binning_x}\nBinning Y: {message.binning_y}");
            message.roi.GUI(m_Tex);
        };
    }
}
