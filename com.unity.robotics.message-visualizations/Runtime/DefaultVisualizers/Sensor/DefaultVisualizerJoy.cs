using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerJoy : VisualFactory<MJoy>
{
    int m_Layout = 0;
    string[] m_SelStrings = {"DS4", "X360 Windows", "X360 Linux", "X360 (Wired)", "F710"};

    public override Action CreateGUI(MJoy message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        
        m_Layout = GUILayout.SelectionGrid(m_Layout, m_SelStrings, 2);

        // Triggers
        GUILayout.BeginHorizontal();
        GUILayout.Box(message.TextureFromJoy(JoyRegion.LT, m_Layout));
        GUILayout.Box(message.TextureFromJoy(JoyRegion.RT, m_Layout));
        GUILayout.EndHorizontal();

        // Shoulders
        GUILayout.BeginHorizontal();
        GUILayout.Box(message.TextureFromJoy(JoyRegion.LB, m_Layout));
        GUILayout.Box(message.TextureFromJoy(JoyRegion.RB, m_Layout));
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();

        // Dpad, central buttons
        GUILayout.Box(message.TextureFromJoy(JoyRegion.DPad, m_Layout));
        GUILayout.Box(message.TextureFromJoy(JoyRegion.Back, m_Layout));
        GUILayout.Box(message.TextureFromJoy(JoyRegion.Power, m_Layout));
        GUILayout.Box(message.TextureFromJoy(JoyRegion.Start, m_Layout));

        // N/E/S/W buttons
        GUILayout.BeginVertical();
        GUILayoutUtility.GetAspectRect(1);
        GUILayout.Box(message.TextureFromJoy(JoyRegion.BWest, m_Layout));
        GUILayoutUtility.GetAspectRect(1);
        GUILayout.EndVertical();
        GUILayout.BeginVertical();
        GUILayout.Box(message.TextureFromJoy(JoyRegion.BNorth, m_Layout));
        GUILayoutUtility.GetAspectRect(1);
        GUILayout.Box(message.TextureFromJoy(JoyRegion.BSouth, m_Layout));
        GUILayout.EndVertical();
        GUILayout.BeginVertical();
        GUILayoutUtility.GetAspectRect(1);
        GUILayout.Box(message.TextureFromJoy(JoyRegion.BEast, m_Layout));
        GUILayoutUtility.GetAspectRect(1);
        GUILayout.EndVertical();

        GUILayout.EndHorizontal();

        // Joysticks
        GUILayout.BeginHorizontal();
        GUILayout.Box(message.TextureFromJoy(JoyRegion.LStick, m_Layout));
        GUILayout.Box(message.TextureFromJoy(JoyRegion.RStick, m_Layout));
        GUILayout.EndHorizontal();
    };
}
