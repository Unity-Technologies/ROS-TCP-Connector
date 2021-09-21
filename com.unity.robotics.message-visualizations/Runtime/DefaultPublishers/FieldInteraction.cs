using System;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class FieldInteraction : Interaction
    {
        Rect m_WindowRect = new Rect(0, 225, 300, 100);
        string m_TextInput;

        void OnGUI()
        {
            Debug.Log(m_State);
            if (m_State != ClickState.None)
            {
                m_WindowRect = GUILayout.Window(0, m_WindowRect, Broadcast, "Publish");
            }
        }

        void Broadcast(int id)
        {
            m_TextInput = GUILayout.TextField(m_TextInput);
            if (GUILayout.Button("Publish"))
            {
                m_RosPublish.PublishString(m_TextInput);
                m_State = ClickState.None;
                m_RosPublish.ResetTrigger();
            }
        }
    }
}
