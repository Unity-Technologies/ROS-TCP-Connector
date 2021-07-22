using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class HudWindow
    {
        const float c_DraggableSize = 8;
        Rect m_WindowRect;
        public Rect WindowRect => m_WindowRect;
        string m_Title;
        bool m_IsActive;
        bool m_AutoLayout;
        bool m_DraggingBottom;
        bool m_DraggingLeft;
        bool m_DraggingRight;
        bool m_DraggingTitle;
        Vector2 m_DragMouseOffset;
        Vector2 m_ContentScrollPosition;
        System.Action m_GuiDrawer;
        int m_WindowID;

        public HudWindow(string title, Rect rect, bool autoLayout = false)
        {
            m_WindowID = HUDPanel.GetNextWindowID();
            m_AutoLayout = autoLayout;
            HUDPanel.AddWindow(this);
        }

        public bool HasActiveRect => m_IsActive && !m_AutoLayout;

        public void SetGuiDrawer(System.Action guiDrawer)
        {
            m_GuiDrawer = guiDrawer;
        }

        public void DrawWindow(HUDPanel hud)
        {
            if (m_IsActive && m_GuiDrawer != null)
            {
                if (m_AutoLayout)
                {
                    m_WindowRect = hud.GetFreeWindowRect();
                    m_AutoLayout = false;
                }
                GUI.Window(m_WindowID, m_WindowRect, DrawWindowContents, m_Title);
            }
        }

        void DrawWindowContents(int id)
        {
            m_ContentScrollPosition = GUILayout.BeginScrollView(m_ContentScrollPosition);
            m_GuiDrawer();
            GUILayout.EndScrollView();
        }

        public bool TryDragWindow(Event current)
        {
            var expandedWindowMenu = new Rect(m_WindowRect.x - c_DraggableSize, m_WindowRect.y, m_WindowRect.width + c_DraggableSize * 2, m_WindowRect.height + c_DraggableSize);
            if (expandedWindowMenu.Contains(current.mousePosition))
            {
                m_DraggingTitle = current.mousePosition.y < m_WindowRect.yMin + c_DraggableSize * 2;
                m_DraggingLeft = current.mousePosition.x < m_WindowRect.xMin + c_DraggableSize;
                m_DraggingRight = current.mousePosition.x > m_WindowRect.xMax - c_DraggableSize;
                m_DraggingBottom = current.mousePosition.y > m_WindowRect.yMax - c_DraggableSize;
            }

            m_DragMouseOffset = current.mousePosition - new Vector2(m_WindowRect.xMin, m_WindowRect.yMin);
            return m_DraggingTitle || m_DraggingLeft || m_DraggingRight || m_DraggingBottom;
        }

        public void UpdateDragWindow(Event current)
        {
            if (m_DraggingTitle)
            {
                m_WindowRect.x = current.mousePosition.x - m_DragMouseOffset.x;
                m_WindowRect.y = current.mousePosition.y - m_DragMouseOffset.y;
            }
            else
            {
                if (m_DraggingLeft)
                    m_WindowRect.xMin = current.mousePosition.x;
                if (m_DraggingBottom)
                    m_WindowRect.yMax = current.mousePosition.y;
                if (m_DraggingRight)
                    m_WindowRect.xMax = current.mousePosition.x;
            }
        }

        public void EndDragWindow()
        {
            m_DraggingTitle = m_DraggingLeft = m_DraggingRight = m_DraggingBottom = false;
        }

    }
}
