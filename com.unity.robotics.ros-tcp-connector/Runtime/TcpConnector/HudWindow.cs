using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public interface IMenuFiller
    {
        void AddItem(GUIContent name, bool selected, System.Action callback);
    }

    public class HudWindow
    {
        const float c_DraggableSize = 8;
        Rect m_WindowRect;
        public Rect WindowRect => m_WindowRect;
        string m_Title;
        bool m_IsActive;
        public bool IsActive => m_IsActive;
        bool m_AutoLayout;
        bool m_DraggingBottom;
        bool m_DraggingLeft;
        bool m_DraggingRight;
        bool m_DraggingTitle;
        Vector2 m_DragMouseOffset;
        Vector2 m_ContentScrollPosition;
        System.Action m_GuiDrawer;
        System.Action<IMenuFiller> m_GuiMenu;
        int m_WindowID;

        public HudWindow(string title, Rect rect)
        {
            m_Title = title;
            m_WindowID = HudPanel.GetNextWindowID();
            m_IsActive = true;
            m_AutoLayout = false;
            m_WindowRect = rect;
        }

        public HudWindow(string title)
        {
            m_Title = title;
            m_WindowID = HudPanel.GetNextWindowID();
            m_IsActive = true;
            m_AutoLayout = true;
        }

        public void SetActive(bool active)
        {
            m_IsActive = active;
        }

        public bool HasActiveRect => m_IsActive && !m_AutoLayout;

        public void SetOnGUI(Action guiDrawer)
        {
            m_GuiDrawer = guiDrawer;
        }

        public void SetGUIMenu(Action<IMenuFiller> guiMenu)
        {
            m_GuiMenu = guiMenu;
        }

        public void DrawWindow(HudPanel hud)
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


#if UNITY_EDITOR
        class EditorMenuBuilder : IMenuFiller
        {
            UnityEditor.GenericMenu m_Menu;
            public EditorMenuBuilder()
            {
                m_Menu = new UnityEditor.GenericMenu();
            }

            public void AddItem(GUIContent name, bool selected, Action callback)
            {
                m_Menu.AddItem(name, selected, ()=>callback());
            }

            public void Show(Vector2 position)
            {
                m_Menu.DropDown(new Rect(position.x, position.y, 0f, 0f));
            }
        }
#endif

        void DrawWindowContents(int id)
        {
            m_ContentScrollPosition = GUILayout.BeginScrollView(m_ContentScrollPosition);
            m_GuiDrawer();
            GUILayout.EndScrollView();
            if (m_GuiMenu != null)
            {
                Rect scrollRect = GUILayoutUtility.GetLastRect();
                if (GUI.Button(new Rect(scrollRect.xMin - 5, scrollRect.yMin - 17, 20, 20), "\u2630"))
                {
#if UNITY_EDITOR
                    EditorMenuBuilder menuBuilder = new EditorMenuBuilder();
                    m_GuiMenu(menuBuilder);
                    menuBuilder.Show(new Vector2(scrollRect.xMin, scrollRect.yMin+70));
#endif
                }
            }
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
