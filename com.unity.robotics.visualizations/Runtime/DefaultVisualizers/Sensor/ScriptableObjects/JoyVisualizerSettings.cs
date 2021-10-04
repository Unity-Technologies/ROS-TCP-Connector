using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.Visualizations;
using UnityEngine;

[CreateAssetMenu(fileName = "JoyVisualizerSettings", menuName = "Robotics/Sensor Visualizers/Joy", order = 1)]
public class JoyVisualizerSettings : ScriptableObject
{
    public enum JoyInputType
    {
        None,
        Axis_0,
        Axis_1,
        Axis_2,
        Axis_3,
        Axis_4,
        Axis_5,
        Axis_6,
        Axis_7,
        Axis_8,
        Axis_9,
        Axis_10,
        Button_0,
        Button_1,
        Button_2,
        Button_3,
        Button_4,
        Button_5,
        Button_6,
        Button_7,
        Button_8,
        Button_9,
        Button_10,
        Button_11,
        Button_12,
        Button_13,
        Button_14,
        Button_15,
    }
    public JoyInputType m_Button_South;
    public JoyInputType m_Button_East;
    public JoyInputType m_Button_West;
    public JoyInputType m_Button_North;
    public JoyInputType m_Button_Back;
    public JoyInputType m_Button_Start;
    public JoyInputType m_Button_Power;
    public JoyInputType m_DPad_Up;
    public JoyInputType m_DPad_Down;
    public JoyInputType m_DPad_Left;
    public JoyInputType m_DPad_Right;
    public JoyInputType m_DPad_XAxis;
    public JoyInputType m_DPad_YAxis;
    public JoyInputType m_LStick_X;
    public JoyInputType m_LStick_Y;
    public JoyInputType m_LStick_Click;
    public JoyInputType m_LShoulder;
    public JoyInputType m_LTrigger;
    public JoyInputType m_RStick_X;
    public JoyInputType m_RStick_Y;
    public JoyInputType m_RStick_Click;
    public JoyInputType m_RShoulder;
    public JoyInputType m_RTrigger;

    public Action CreateGUI(JoyMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();

            Rect layout = GUILayoutUtility.GetAspectRect(2);
            float minX = layout.xMin;
            float maxX = layout.xMax;
            float midX = (minX + maxX) / 2;
            float minY = layout.yMin;
            float maxY = layout.yMax;
            float midY = (minY + maxY) / 2;
            float w = layout.width;
            float h = layout.height;
            float sw = w * 0.125f;
            float sh = h * 0.25f;
            float bw = w * 0.0625f;
            float bh = h * 0.125f;

            // Triggers
            GUI.Box(new Rect(minX + sw, minY, bw, sh), TriggerTexture(GetInputFloat(message, m_LTrigger)));
            GUI.Box(new Rect(maxX - sw * 1.5f, minY, bw, sh), TriggerTexture(GetInputFloat(message, m_RTrigger)));

            // Shoulders
            GUI.Box(new Rect(minX + sw * 1.5f, minY + bh, bw, bh), ButtonTexture(GetInputBool(message, m_LShoulder)));
            GUI.Box(new Rect(maxX - sw * 2.0f, minY + bh, bw, bh), ButtonTexture(GetInputBool(message, m_RShoulder)));

            // Dpad, central buttons
            GUI.Box(new Rect(midX - sw * 2.5f, maxY - sh, sw, sh),
                StickTexture(
                GetInputAxisOrButtons(message, m_DPad_XAxis, m_DPad_Left, m_DPad_Right),
                GetInputAxisOrButtons(message, m_DPad_YAxis, m_DPad_Up, m_DPad_Down),
                false)
            );

            if (m_Button_Back != JoyInputType.None)
                GUI.Box(new Rect(midX - bw * 2.5f, midY - bh, bw, bh), ButtonTexture(GetInputBool(message, m_Button_Back)));
            if (m_Button_Power != JoyInputType.None)
                GUI.Box(new Rect(midX - bw * 0.5f, midY - bh, bw, bh), ButtonTexture(GetInputBool(message, m_Button_Power)));
            if (m_Button_Start != JoyInputType.None)
                GUI.Box(new Rect(midX + bw * 1.5f, midY - bh, bw, bh), ButtonTexture(GetInputBool(message, m_Button_Start)));

            // N/E/S/W buttons
            float btnX = maxX - sw * 1.25f;
            float btnY = midY - bh * 0.5f;
            GUI.Box(new Rect(btnX - bw, btnY, bw, bh), ButtonTexture(GetInputBool(message, m_Button_West)));
            GUI.Box(new Rect(btnX, btnY - bh, bw, bh), ButtonTexture(GetInputBool(message, m_Button_North)));
            GUI.Box(new Rect(btnX, btnY + bh, bw, bh), ButtonTexture(GetInputBool(message, m_Button_South)));
            GUI.Box(new Rect(btnX + bw, btnY, bw, bh), ButtonTexture(GetInputBool(message, m_Button_East)));

            // Joysticks
            GUI.Box(new Rect(minX + sw * 0.5f, midY - sh * 0.5f, sw, sh), StickTexture(GetInputFloat(message, m_LStick_X), GetInputFloat(message, m_LStick_Y), GetInputBool(message, m_LStick_Click)));
            GUI.Box(new Rect(midX + sw * 1.5f, maxY - sh, sw, sh), StickTexture(GetInputFloat(message, m_RStick_X), GetInputFloat(message, m_RStick_Y), GetInputBool(message, m_RStick_Click)));
        };
    }

    public float GetInputFloat(JoyMsg message, JoyInputType inputType)
    {
        if (inputType >= JoyInputType.Button_0)
        {
            int buttonIndex = inputType - JoyInputType.Button_0;
            if (buttonIndex < 0 || buttonIndex >= message.buttons.Length)
                return 0.0f;
            return message.buttons[buttonIndex] != 0 ? 1.0f : 0.0f;
        }

        int axisIndex = inputType - JoyInputType.Axis_0;
        if (axisIndex < 0 || axisIndex >= message.axes.Length)
            return 0.0f;
        return message.axes[axisIndex];
    }

    public float GetInputAxisOrButtons(JoyMsg message, JoyInputType axis, JoyInputType upButton, JoyInputType downButton)
    {
        if (axis != JoyInputType.None)
            return GetInputFloat(message, axis);

        return GetInputFloat(message, upButton) - GetInputFloat(message, downButton);
    }

    public bool GetInputBool(JoyMsg message, JoyInputType inputType)
    {
        if (inputType >= JoyInputType.Button_0)
        {
            int buttonIndex = inputType - JoyInputType.Button_0;
            if (buttonIndex < 0 || buttonIndex >= message.buttons.Length)
                return false;
            return message.buttons[buttonIndex] != 0;
        }

        int axisIndex = inputType - JoyInputType.Axis_0;
        if (axisIndex < 0 || axisIndex >= message.axes.Length)
            return false;
        return message.axes[axisIndex] != 0;
    }

    static Color32[] s_ColorHighlight;
    static Color32[] GetColorHighlight()
    {
        if (s_ColorHighlight == null)
        {
            s_ColorHighlight = new Color32[100];
            for (int i = 0; i < s_ColorHighlight.Length; i++)
            {
                s_ColorHighlight[i] = Color.red;
            }
        }
        return s_ColorHighlight;
    }

    static Color32[] s_ColorPress;
    static Color32[] GetColorPress()
    {
        if (s_ColorPress == null)
        {
            s_ColorPress = new Color32[2500];
            for (int i = 0; i < s_ColorPress.Length; i++)
            {
                s_ColorPress[i] = Color.blue;
            }
        }
        return s_ColorPress;
    }

    public Texture2D TriggerTexture(float fraction)
    {
        Texture2D tex = new Texture2D(25, 50);
        int y = Mathf.FloorToInt(fraction * 20) + tex.height / 2;
        tex.SetPixels32(0, y - 2, 25, 4, GetColorHighlight());
        tex.Apply();
        return tex;
    }

    static Texture2D s_ButtonPressed;
    static Texture2D s_ButtonUnpressed;

    public Texture2D ButtonTexture(bool pressed)
    {
        if (pressed)
        {
            if (s_ButtonPressed == null)
            {
                s_ButtonPressed = new Texture2D(10, 10);
                s_ButtonPressed.SetPixels32(GetColorHighlight());
                s_ButtonPressed.Apply();
            }
            return s_ButtonPressed;
        }
        else
        {
            if (s_ButtonUnpressed == null)
            {
                s_ButtonUnpressed = new Texture2D(10, 10);
                s_ButtonUnpressed.Apply();
            }
            return s_ButtonUnpressed;
        }
    }

    public Texture2D StickTexture(float xAxis, float yAxis, bool clicked)
    {
        Texture2D tex = new Texture2D(50, 50);
        if (clicked)
            tex.SetPixels32(GetColorPress());
        int x = Mathf.FloorToInt(xAxis * -20) + tex.width / 2;
        int y = Mathf.FloorToInt(yAxis * 20) + tex.height / 2;
        tex.SetPixels32(x - 5, y - 5, 10, 10, GetColorHighlight());
        tex.Apply();
        return tex;
    }
}
