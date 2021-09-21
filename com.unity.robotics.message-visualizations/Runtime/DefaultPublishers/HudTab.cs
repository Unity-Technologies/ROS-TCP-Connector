using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class HudTab : MonoBehaviour, IHudTab
    {
        [SerializeField]
        string m_Label;
        [SerializeField]
        List<HudButton> m_Buttons = new List<HudButton>();

        public void Start()
        {
            HudPanel.RegisterTab(this);
        }

        string IHudTab.Label => m_Label;

        void IHudTab.OnGUI(HudPanel hud)
        {
            GUILayout.BeginHorizontal();
            foreach (var b in m_Buttons)
            {
                b.DrawGUI();
            }
            GUILayout.EndHorizontal();
        }

        void IHudTab.OnSelected() { }

        void IHudTab.OnDeselected() { }
    }
}
