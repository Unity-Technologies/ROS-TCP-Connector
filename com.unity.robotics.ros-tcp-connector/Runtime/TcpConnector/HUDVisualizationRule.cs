using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class HUDVisualizationRule
    {
        public string Topic { get; private set; }
        public string RosMessageName { get; private set; }
        public bool ShowWindow { get; private set; }
        public bool ShowDrawing { get; private set; }
        public Vector2 WindowScrollPosition { get; set; }

        HUDPanel m_Hud;
        IWindowContents m_Contents;

        Rect m_WindowRect;
        public Rect WindowRect => m_WindowRect;
        bool m_HasWindowRect;
        int m_WindowID;
        int m_ServiceID;

        const float c_DraggableSize = 8;
        Vector2 m_DragMouseOffset;
        bool m_DraggingTitle;
        bool m_DraggingLeft;
        bool m_DraggingRight;
        bool m_DraggingBottom;

        [Serializable]
        public class SaveState
        {
            public Rect Rect;
            public string Topic;
            public string RosMessageName;
            public bool ShowWindow;
            public bool ShowDrawing;
        }

        public HUDVisualizationRule(SaveState saveState, HUDPanel hud)
        {
            m_Hud = hud;
            m_WindowRect = saveState.Rect;
            m_HasWindowRect = saveState.ShowWindow;
            m_WindowID = hud.GetNextWindowID();
            Topic = saveState.Topic;
            RosMessageName = saveState.RosMessageName;
            m_Contents = new MessageWindowContents(this, Topic);
            if (!ROSConnection.instance.HasSubscriber(Topic))
            {
                ROSConnection.instance.SubscribeByMessageName(Topic, saveState.RosMessageName, (Message) => { });
                ROSConnection.instance.RegisterSubscriber(Topic, saveState.RosMessageName);
            }
            SetShowWindow(saveState.ShowWindow);
            SetShowDrawing(saveState.ShowDrawing);
        }

        public HUDVisualizationRule(string topic, string rosMessageName, HUDPanel hud)
        {
            m_Hud = hud;
            m_WindowRect = HUDPanel.GetDefaultWindowRect();
            m_WindowID = hud.GetNextWindowID();
            Topic = topic;
            RosMessageName = rosMessageName;
        }

        public SaveState CreateSaveState()
        {
            if (!ShowWindow && !ShowDrawing)
                return null;

            return new SaveState
            {
                Rect = m_WindowRect,
                Topic = Topic,
                RosMessageName = RosMessageName,
                ShowWindow = ShowWindow,
                ShowDrawing = ShowDrawing,
            };
        }

        public void SetMessage(Message message, MessageMetadata meta)
        {
            if (m_Contents is MessageWindowContents)
                ((MessageWindowContents)m_Contents).SetMessage(message, meta);
            else
                m_Contents = new MessageWindowContents(this, message, meta);

            if (ShowDrawing)
                m_Contents.ShowDrawing(true);
        }

        public void SetServiceRequest(Message request, MessageMetadata requestMeta, int serviceID)
        {
            this.m_ServiceID = serviceID;
            if (m_Contents is ServiceWindowContents)
                ((ServiceWindowContents)m_Contents).SetRequest(request, requestMeta);
            else
                m_Contents = new ServiceWindowContents(this, request, requestMeta);

            if (ShowDrawing)
                m_Contents.ShowDrawing(true);
        }

        public void SetServiceResponse(Message response, MessageMetadata responseMeta, int serviceID)
        {
            // If this is not a response to the request we have, ignore it.
            // TODO: need more granular control over this
            if (this.m_ServiceID != serviceID)
                return;

            if (m_Contents is ServiceWindowContents)
            {
                ((ServiceWindowContents)m_Contents).SetResponse(response, responseMeta);
                if (ShowDrawing)
                    m_Contents.ShowDrawing(true);
            }
        }

        public void DrawWindow()
        {
            if (m_Contents != null)
                m_Contents.DrawWindow(m_WindowID, m_WindowRect);
        }

        public void SetShowDrawing(bool showDrawing)
        {
            bool hasDrawing = m_Contents != null && m_Contents.HasDrawing;
            this.ShowDrawing = showDrawing;
            if (showDrawing != hasDrawing && m_Contents != null)
            {
                m_Contents.ShowDrawing(showDrawing);
            }
        }

        public void SetShowWindow(bool showWindow)
        {
            bool hasWindow = this.ShowWindow;
            if (showWindow == hasWindow)
                return;

            if (!showWindow)
            {
                this.ShowWindow = false;
                m_Hud.RemoveWindow(this);
                return;
            }

            if (m_Contents == null)
            {
                string rosMessageName = HUDPanel.GetMessageNameByTopic(Topic);
                if (ROSConnection.instance.HasSubscriber(Topic))
                {
                    m_Contents = new MessageWindowContents(this, Topic);
                }
                else
                {
                    Func<Message> messageConstructor = MessageRegistry.GetConstructor(rosMessageName);
                    if (messageConstructor == null)
                    {
                        Debug.LogError("No known message class for " + rosMessageName);
                        return;
                    }
                    else
                    {
                        ROSConnection.instance.RegisterSubscriber(Topic, rosMessageName);
                        // TODO: this should not be necessary
                        ROSConnection.instance.SubscribeByMessageName(Topic, rosMessageName, (Message m) => { });
                        m_Contents = new MessageWindowContents(this, Topic);
                    }
                }
            }

            this.ShowWindow = true;
            if (!m_HasWindowRect || !m_Hud.IsFreeWindowRect(m_WindowRect))
            {
                m_WindowRect = m_Hud.GetFreeWindowRect();
                m_HasWindowRect = true;
            }
            m_Hud.AddWindow(this);
        }

        public bool TryDragWindow(Event current)
        {
            Rect expandedWindowMenu = new Rect(m_WindowRect.x - c_DraggableSize, m_WindowRect.y, m_WindowRect.width + c_DraggableSize * 2, m_WindowRect.height + c_DraggableSize);
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

        interface IWindowContents
        {
            bool HasDrawing { get; }
            void ShowDrawing(bool show);
            void DrawWindow(int windowID, Rect windowRect);
        }

        class MessageWindowContents : IWindowContents
        {
            HUDVisualizationRule m_Rule;
            Message m_Message;
            MessageMetadata m_Meta;
            IVisualizer m_VisualizerConfig;
            object m_VisualizerDrawing;
            Action m_VisualizerGUI;

            public bool HasDrawing => m_VisualizerDrawing != null;

            public MessageWindowContents(HUDVisualizationRule rule, Message message, MessageMetadata meta)
            {
                m_Rule = rule;
                m_Message = message;
                m_Meta = meta;
            }

            public MessageWindowContents(HUDVisualizationRule rule, string topic)
            {
                m_Rule = rule;
                m_Message = null;
                m_Meta = new MessageMetadata(topic, DateTime.Now);
            }

            public void SetMessage(Message message, MessageMetadata meta)
            {
                m_Message = message;
                m_Meta = meta;
                m_VisualizerGUI = null;
            }

            public void ShowDrawing(bool show)
            {
                if (show)
                {
                    if (m_VisualizerConfig == null && m_Message != null)
                        m_VisualizerConfig = VisualizationRegistry.GetVisualizer(m_Message, m_Meta);

                    if (m_VisualizerConfig != null)
                    {
                        object oldDrawing = m_VisualizerDrawing;
                        m_VisualizerDrawing = m_VisualizerConfig.CreateDrawing(m_Message, m_Meta, oldDrawing);

                        if (oldDrawing != null && oldDrawing != m_VisualizerDrawing)
                            m_VisualizerConfig.DeleteDrawing(oldDrawing);
                    }
                }
                else
                {
                    if (m_VisualizerConfig != null && m_VisualizerDrawing != null)
                        m_VisualizerConfig.DeleteDrawing(m_VisualizerDrawing);
                    m_VisualizerDrawing = null;
                }
            }

            public void DrawWindow(int windowID, Rect windowRect)
            {
                GUI.Window(windowID, windowRect, DrawWindowContents, m_Meta.Topic);
            }

            void DrawWindowContents(int id)
            {
                if (m_Message == null)
                {
                    GUILayout.Label("Waiting for message...");
                }
                else
                {
                    if (m_VisualizerConfig == null)
                        m_VisualizerConfig = VisualizationRegistry.GetVisualizer(m_Message, m_Meta);

                    if (m_VisualizerGUI == null)
                        m_VisualizerGUI = m_VisualizerConfig.CreateGUI(m_Message, m_Meta, m_VisualizerDrawing);

                    m_Rule.WindowScrollPosition = GUILayout.BeginScrollView(m_Rule.WindowScrollPosition);
                    m_VisualizerGUI();
                    GUILayout.EndScrollView();
                }
            }
        }

        class ServiceWindowContents : IWindowContents
        {
            HUDVisualizationRule m_Rule;
            Message m_Request;
            MessageMetadata m_RequestMeta;
            IVisualizer m_RequestVisualizer;
            object m_RequestDrawing;
            Action m_RequestGUI;

            Message m_Response;
            MessageMetadata m_ResponseMeta;
            IVisualizer m_ResponseVisualizer;
            object m_ResponseDrawing;
            Action m_ResponseGUI;

            public bool HasDrawing => m_RequestDrawing != null || m_ResponseDrawing != null;

            public ServiceWindowContents(HUDVisualizationRule rule, Message request, MessageMetadata requestMeta)
            {
                m_Rule = rule;
                m_Request = request;
                m_RequestMeta = requestMeta;
            }

            public void SetRequest(Message request, MessageMetadata requestMeta)
            {
                m_Request = request;
                m_RequestMeta = requestMeta;
                m_RequestGUI = null;
            }

            public void SetResponse(Message response, MessageMetadata responseMeta)
            {
                m_Response = response;
                m_ResponseMeta = responseMeta;
                m_ResponseGUI = null;
            }

            public void ShowDrawing(bool show)
            {
                if (show)
                {
                    if (m_RequestVisualizer == null && m_Request != null)
                        m_RequestVisualizer = VisualizationRegistry.GetVisualizer(m_Request, m_RequestMeta);

                    if (m_ResponseVisualizer == null && m_Response != null)
                        m_ResponseVisualizer = VisualizationRegistry.GetVisualizer(m_Response, m_ResponseMeta);

                    if (m_RequestVisualizer != null && m_RequestDrawing == null)
                        m_RequestDrawing = m_RequestVisualizer.CreateDrawing(m_Request, m_RequestMeta, m_RequestDrawing);

                    if (m_ResponseVisualizer != null && m_ResponseDrawing == null)
                        m_ResponseDrawing = m_ResponseVisualizer.CreateDrawing(m_Response, m_ResponseMeta, m_ResponseDrawing);
                }
                else
                {
                    if (m_RequestVisualizer != null && m_RequestDrawing != null)
                        m_RequestVisualizer.DeleteDrawing(m_RequestDrawing);
                    m_RequestDrawing = null;

                    if (m_ResponseVisualizer != null && m_ResponseDrawing != null)
                        m_ResponseVisualizer.DeleteDrawing(m_ResponseDrawing);
                    m_ResponseDrawing = null;
                }
            }

            public void DrawWindow(int windowID, Rect windowRect)
            {
                GUI.Window(windowID, windowRect, DrawWindowContents, m_RequestMeta.Topic);
            }

            void DrawWindowContents(int id)
            {
                m_Rule.WindowScrollPosition = GUILayout.BeginScrollView(m_Rule.WindowScrollPosition);

                if (m_Request == null)
                {
                    GUILayout.Label("Waiting for request...");
                }
                else
                {
                    if (m_RequestVisualizer == null)
                        m_RequestVisualizer = VisualizationRegistry.GetVisualizer(m_Request, m_RequestMeta);

                    if (m_RequestGUI == null)
                        m_RequestGUI = m_RequestVisualizer.CreateGUI(m_Request, m_RequestMeta, m_RequestDrawing);

                    m_RequestGUI();
                }

                // horizontal line
                GUILayout.Label("", GUI.skin.horizontalSlider);

                if (m_Response == null)
                {
                    GUILayout.Label("Waiting for response...");
                }
                else
                {
                    if (m_ResponseVisualizer == null)
                        m_ResponseVisualizer = VisualizationRegistry.GetVisualizer(m_Response, m_ResponseMeta);

                    if (m_ResponseGUI == null)
                        m_ResponseGUI = m_ResponseVisualizer.CreateGUI(m_Response, m_ResponseMeta, m_ResponseDrawing);

                    m_ResponseGUI();
                }
                GUILayout.EndScrollView();
            }
        }
    }
}