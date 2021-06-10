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
        int m_DrawingUpdatedAtFrameIndex;

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

            if (ShowDrawing && m_DrawingUpdatedAtFrameIndex != meta.FrameIndex)
            {
                m_Contents.ShowDrawing(true);
                m_DrawingUpdatedAtFrameIndex = meta.FrameIndex;
            }
        }

        public void SetServiceRequest(Message request, MessageMetadata requestMeta, int serviceID)
        {
            this.m_ServiceID = serviceID;
            if (m_Contents is ServiceWindowContents)
                ((ServiceWindowContents)m_Contents).SetRequest(request, requestMeta);
            else
                m_Contents = new ServiceWindowContents(this, request, requestMeta);

            if (ShowDrawing && m_DrawingUpdatedAtFrameIndex != requestMeta.FrameIndex)
            {
                m_Contents.ShowDrawing(true);
                m_DrawingUpdatedAtFrameIndex = requestMeta.FrameIndex;
            }
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
                if (ShowDrawing && m_DrawingUpdatedAtFrameIndex != responseMeta.FrameIndex)
                {
                    m_Contents.ShowDrawing(true);
                    m_DrawingUpdatedAtFrameIndex = responseMeta.FrameIndex;
                }
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

            if (showDrawing != hasDrawing)
            {
                if (showDrawing && m_Contents == null)
                {
                    if (TrySubscribe())
                        m_Contents = new MessageWindowContents(this, Topic);
                }

                if (m_Contents != null)
                {
                    m_Contents.ShowDrawing(showDrawing);
                }
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

            if (m_Contents == null && TrySubscribe())
            {
                m_Contents = new MessageWindowContents(this, Topic);
            }

            this.ShowWindow = true;
            if (!m_HasWindowRect || !m_Hud.IsFreeWindowRect(m_WindowRect))
            {
                m_WindowRect = m_Hud.GetFreeWindowRect();
                m_HasWindowRect = true;
            }
            m_Hud.AddWindow(this);
        }

        bool TrySubscribe()
        {
            string rosMessageName = HUDPanel.GetMessageNameByTopic(Topic);
            if (!ROSConnection.instance.HasSubscriber(Topic))
            {
                Func<Message> messageConstructor = MessageRegistry.GetConstructor(rosMessageName);
                if (messageConstructor == null)
                {
                    Debug.LogError("No known message class for " + rosMessageName);
                    return false;
                }

                ROSConnection.instance.RegisterSubscriber(Topic, rosMessageName);
                // TODO: this should not be necessary
                ROSConnection.instance.SubscribeByMessageName(Topic, rosMessageName, (Message m) => { });
            }

            return true;
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
            IMessageVisualization m_Visualization;

            public bool HasDrawing => m_Visualization != null && m_Visualization.hasDrawing;

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
                m_Meta = new MessageMetadata(topic, 0, DateTime.Now);
            }

            public void SetMessage(Message message, MessageMetadata meta)
            {
                m_Message = message;
                m_Meta = meta;
                if (m_Visualization != null)
                {
                    m_Visualization.hasAction = false;
                }
            }

            public void ShowDrawing(bool show)
            {
                if (show)
                {
                    if (m_VisualizerConfig == null && m_Message != null)
                    {
                        m_VisualizerConfig = VisualizationRegistry.GetVisualizer(m_Message, m_Meta);
                    }
                    
                    if (m_VisualizerConfig != null)
                    {
                        m_Visualization?.Delete();
                        m_Visualization = m_VisualizerConfig.CreateVisualization(m_Message, m_Meta, false, true);
                    }
                }
                else
                {
                    if (m_VisualizerConfig != null && m_Visualization.hasDrawing)
                    {
                        m_Visualization.Delete();
                    }
                    m_Visualization.hasDrawing = false;
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
                    {
                        m_VisualizerConfig = VisualizationRegistry.GetVisualizer(m_Message, m_Meta);
                    }
                    if (m_VisualizerConfig != null)
                    {
                        m_Visualization?.Delete();
                        m_Visualization = m_VisualizerConfig.CreateVisualization(m_Message, m_Meta, true, false);
                    }
                    

                    m_Rule.WindowScrollPosition = GUILayout.BeginScrollView(m_Rule.WindowScrollPosition);
                    m_Visualization.OnGUI();
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
            IMessageVisualization m_RequestVisualization;

            Message m_Response;
            MessageMetadata m_ResponseMeta;
            IVisualizer m_ResponseVisualizer;
            IMessageVisualization m_ResponseVisualization;

            public bool HasDrawing => (m_RequestVisualization != null && m_RequestVisualization.hasDrawing) || (m_ResponseVisualization != null && m_ResponseVisualization.hasDrawing);

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
                m_RequestVisualization.hasAction = false;
            }

            public void SetResponse(Message response, MessageMetadata responseMeta)
            {
                m_Response = response;
                m_ResponseMeta = responseMeta;
                m_ResponseVisualization.hasAction = false;
            }

            public void ShowDrawing(bool show)
            {
                if (show)
                {
                    if (m_RequestVisualizer == null && m_Request != null)
                    {
                        m_RequestVisualizer = VisualizationRegistry.GetVisualizer(m_Request, m_RequestMeta);
                    }

                    if (m_ResponseVisualizer == null && m_Response != null)
                    {
                        m_ResponseVisualizer = VisualizationRegistry.GetVisualizer(m_Response, m_ResponseMeta);
                    }

                    if (m_RequestVisualizer != null && !m_RequestVisualization.hasDrawing)
                    {
                        m_RequestVisualization?.Delete();
                        m_RequestVisualization = m_RequestVisualizer.CreateVisualization(m_Request, m_RequestMeta, false, true);
                    }

                    if (m_ResponseVisualizer != null && !m_ResponseVisualization.hasDrawing)
                    {
                        m_ResponseVisualization?.Delete();
                        m_ResponseVisualization = m_ResponseVisualizer.CreateVisualization(m_Response, m_ResponseMeta, false, true);
                    }
                }
                else
                {
                    if (m_RequestVisualizer != null && m_RequestVisualization.hasDrawing)
                    {
                        m_RequestVisualization.Delete();
                    }
                    m_RequestVisualization.hasDrawing = false;

                    if (m_ResponseVisualizer != null && m_ResponseVisualization.hasDrawing)
                    {
                        m_ResponseVisualization.Delete();
                    }
                    m_ResponseVisualization.hasDrawing = false;

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
                    {
                        m_RequestVisualizer = VisualizationRegistry.GetVisualizer(m_Request, m_RequestMeta);
                    }

                    if (!m_RequestVisualization.hasAction)
                    {
                        m_RequestVisualization?.Delete();
                        m_RequestVisualization = m_RequestVisualizer.CreateVisualization(m_Response, m_ResponseMeta, true, false);
                    }

                    m_RequestVisualization.OnGUI();
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
                    {
                        m_ResponseVisualizer = VisualizationRegistry.GetVisualizer(m_Response, m_ResponseMeta);
                    }

                    if (!m_ResponseVisualization.hasAction)
                    {
                        m_ResponseVisualization?.Delete();
                        m_ResponseVisualization = m_ResponseVisualizer.CreateVisualization(m_Response, m_ResponseMeta, true, false);
                    }

                    m_ResponseVisualization.OnGUI();
                }
                GUILayout.EndScrollView();
            }
        }
    }
}