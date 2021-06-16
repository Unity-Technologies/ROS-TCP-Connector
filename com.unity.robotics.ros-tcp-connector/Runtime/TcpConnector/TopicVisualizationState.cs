using System;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class TopicVisualizationState
    {
        const float c_DraggableSize = 8;
        IWindowContents m_Contents;
        bool m_DraggingBottom;
        bool m_DraggingLeft;
        bool m_DraggingRight;
        bool m_DraggingTitle;
        Vector2 m_DragMouseOffset;
        int m_DrawingUpdatedAtFrameIndex;
        bool m_HasWindowRect;

        HUDPanel m_Hud;
        int m_ServiceID;
        int m_WindowID;

        Rect m_WindowRect;

        public TopicVisualizationState(SaveState saveState, HUDPanel hud)
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
                ROSConnection.instance.SubscribeByMessageName(Topic, saveState.RosMessageName, Message => { });
                ROSConnection.instance.RegisterSubscriber(Topic, saveState.RosMessageName);
            }

            SetShowWindow(saveState.ShowWindow);
            SetShowDrawing(saveState.ShowDrawing);
        }

        public TopicVisualizationState(string topic, string rosMessageName, HUDPanel hud, bool subscribe=false)
        {
            m_Hud = hud;
            m_WindowRect = HUDPanel.GetDefaultWindowRect();
            m_HasWindowRect = false;
            m_WindowID = hud.GetNextWindowID();
            Topic = topic;
            RosMessageName = rosMessageName;
            if (subscribe)
            {
                if (!ROSConnection.instance.HasSubscriber(Topic))
                {
                    ROSConnection.instance.SubscribeByMessageName(Topic, RosMessageName, Message => { });
                    ROSConnection.instance.RegisterSubscriber(Topic, RosMessageName);
                }

                SetShowWindow(false);
                SetShowDrawing(false);
            }
        }

        public string Topic { get; }
        public string RosMessageName { get; }
        public bool ShowWindow { get; private set; }
        public bool ShowDrawing { get; private set; }
        public Vector2 WindowScrollPosition { get; set; }
        public Rect WindowRect => m_WindowRect;
        public MessageWindowContents WindowContents => m_Contents as MessageWindowContents;

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
            {
                ((MessageWindowContents)m_Contents).SetMessage(message, meta);
            }
            else
            {
                m_Contents = new MessageWindowContents(this, message, meta);
            }

            if (m_DrawingUpdatedAtFrameIndex != meta.FrameIndex)
            {
                m_Contents.ClearVisual();
                if (ShowDrawing)
                {
                    m_Contents.ShowDrawing(true);
                }
                
                // if (GetVisual) m_Contents.GetVisual();
                m_DrawingUpdatedAtFrameIndex = meta.FrameIndex;
            }
            
        }

        public void SetServiceRequest(Message request, MessageMetadata requestMeta, int serviceID)
        {
            m_ServiceID = serviceID;
            if (m_Contents is ServiceWindowContents)
                ((ServiceWindowContents)m_Contents).SetRequest(request, requestMeta);
            else
                m_Contents = new ServiceWindowContents(this, request, requestMeta);

            if (m_DrawingUpdatedAtFrameIndex != requestMeta.FrameIndex)
            {
                if (ShowDrawing) m_Contents.ShowDrawing(true);
                m_DrawingUpdatedAtFrameIndex = requestMeta.FrameIndex;
            }
        }

        public void SetServiceResponse(Message response, MessageMetadata responseMeta, int serviceID)
        {
            // If this is not a response to the request we have, ignore it.
            // TODO: need more granular control over this
            if (m_ServiceID != serviceID)
                return;

            if (m_Contents is ServiceWindowContents)
            {
                ((ServiceWindowContents)m_Contents).SetResponse(response, responseMeta);
                if (m_DrawingUpdatedAtFrameIndex != responseMeta.FrameIndex)
                {
                    if (ShowDrawing) m_Contents.ShowDrawing(true);
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
            var hasDrawing = m_Contents != null && m_Contents.HasDrawing;
            ShowDrawing = showDrawing;

            if (showDrawing != hasDrawing)
            {
                if (showDrawing && m_Contents == null)
                    if (TrySubscribe())
                        m_Contents = new MessageWindowContents(this, Topic);

                if (m_Contents != null) m_Contents.ShowDrawing(showDrawing);
            }
        }

        public void SetShowWindow(bool showWindow)
        {
            var hasWindow = ShowWindow;
            if (showWindow == hasWindow)
                return;

            if (!showWindow)
            {
                ShowWindow = false;
                m_Hud.RemoveWindow(this);
                return;
            }

            if (m_Contents == null && TrySubscribe()) m_Contents = new MessageWindowContents(this, Topic);

            ShowWindow = true;
            if (!m_HasWindowRect || !m_Hud.IsFreeWindowRect(m_WindowRect))
            {
                m_WindowRect = m_Hud.GetFreeWindowRect();
                m_HasWindowRect = true;
            }

            m_Hud.AddWindow(this);
        }

        bool TrySubscribe()
        {
            var rosMessageName = HUDPanel.GetMessageNameByTopic(Topic);
            if (!ROSConnection.instance.HasSubscriber(Topic))
            {
                var messageConstructor = MessageRegistry.GetConstructor(rosMessageName);
                if (messageConstructor == null)
                {
                    Debug.LogError("No known message class for " + rosMessageName);
                    return false;
                }

                ROSConnection.instance.RegisterSubscriber(Topic, rosMessageName);

                // TODO: this should not be necessary
                ROSConnection.instance.SubscribeByMessageName(Topic, rosMessageName, m => { });
            }
            return true;
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

        [Serializable]
        public class SaveState
        {
            public Rect Rect;
            public string Topic;
            public string RosMessageName;
            public bool ShowWindow;
            public bool ShowDrawing;
        }

        public interface IWindowContents
        {
            bool HasDrawing { get; }
            void ShowDrawing(bool show);
            void DrawWindow(int windowID, Rect windowRect);
            IVisual GetVisual();
            void ClearVisual();
        }

        public class MessageWindowContents : IWindowContents
        {
            Message m_Message;
            MessageMetadata m_Meta;
            TopicVisualizationState m_State;
            IVisual m_Visual;
            IVisualFactory m_VisualFactory;

            public MessageWindowContents(TopicVisualizationState state, Message message, MessageMetadata meta)
            {
                m_Message = message;
                m_Meta = meta;
                m_State = state;
            }

            public MessageWindowContents(TopicVisualizationState state, string topic)
            {
                m_Message = null;
                m_Meta = new MessageMetadata(topic, 0, DateTime.Now);
                m_State = state;
            }

            public bool HasDrawing => m_Visual != null && m_Visual.hasDrawing;

            public void ShowDrawing(bool show)
            {
                if (show)
                {
                    if (m_VisualFactory == null && m_Message != null) m_VisualFactory = VisualFactoryRegistry.GetVisualizer(m_Message, m_Meta);

                    if (m_VisualFactory != null)
                    {
                        ClearVisual();
                        m_Visual = m_VisualFactory.CreateVisual(m_Message, m_Meta);    
                        m_Visual.CreateDrawing();
                    }
                }
                else
                {
                    if (m_Visual != null) m_Visual.DeleteDrawing();
                    // m_Visual.hasDrawing = false;
                }
            }

            public void DrawWindow(int windowID, Rect windowRect)
            {
                GUI.Window(windowID, windowRect, DrawWindowContents, m_Meta.Topic);
            }

            public IVisual GetVisual()
            {
                if (m_Visual == null)
                {
                    if (m_VisualFactory == null && m_Message != null) m_VisualFactory = VisualFactoryRegistry.GetVisualizer(m_Message, m_Meta);

                    if (m_VisualFactory != null)
                    {
                        m_Visual = m_VisualFactory.CreateVisual(m_Message, m_Meta);
                    }
                }   

                return m_Visual;
            }

            public void ClearVisual()
            {
                m_Visual?.DeleteDrawing();
                m_Visual = null;
            }

            public void SetMessage(Message message, MessageMetadata meta)
            {
                m_Message = message;
                m_Meta = meta;
            }

            void DrawWindowContents(int id)
            {
                if (m_Message == null)
                {
                    GUILayout.Label("Waiting for message...");
                }
                else
                {
                    if (m_VisualFactory == null) m_VisualFactory = VisualFactoryRegistry.GetVisualizer(m_Message, m_Meta);

                    m_State.WindowScrollPosition = GUILayout.BeginScrollView(m_State.WindowScrollPosition);
                    if (m_Visual == null)
                    {
                        m_Visual = m_VisualFactory.CreateVisual(m_Message, m_Meta);
                    }
                    m_Visual.OnGUI();
                    GUILayout.EndScrollView();
                }
            }
        }

        class ServiceWindowContents : IWindowContents
        {
            Message m_Request;
            MessageMetadata m_RequestMeta;
            IVisual m_RequestVisual;
            IVisualFactory m_RequestVisualFactory;

            Message m_Response;
            MessageMetadata m_ResponseMeta;
            IVisual m_ResponseVisual;
            IVisualFactory m_ResponseVisualFactory;
            TopicVisualizationState m_State;

            public ServiceWindowContents(TopicVisualizationState state, Message request, MessageMetadata requestMeta)
            {
                m_State = state;
                m_Request = request;
                m_RequestMeta = requestMeta;
            }

            public bool HasDrawing => m_RequestVisual != null && m_RequestVisual.hasDrawing || m_ResponseVisual != null && m_ResponseVisual.hasDrawing;

            public void ShowDrawing(bool show)
            {
                if (show)
                {
                    if (m_RequestVisualFactory == null && m_Request != null) m_RequestVisualFactory = VisualFactoryRegistry.GetVisualizer(m_Request, m_RequestMeta);

                    if (m_ResponseVisualFactory == null && m_Response != null) m_ResponseVisualFactory = VisualFactoryRegistry.GetVisualizer(m_Response, m_ResponseMeta);

                    if (m_RequestVisualFactory != null && !m_RequestVisual.hasDrawing)
                    {
                        m_RequestVisual?.DeleteDrawing();
                        if (m_RequestVisual == null)
                        {
                            m_RequestVisual = m_RequestVisualFactory.CreateVisual(m_Request, m_RequestMeta);
                        }
                        m_RequestVisual.CreateDrawing();
                    }

                    if (m_ResponseVisualFactory != null && !m_ResponseVisual.hasDrawing)
                    {
                        m_ResponseVisual?.DeleteDrawing();
                        if (m_ResponseVisual == null)
                        {
                            m_ResponseVisual = m_ResponseVisualFactory.CreateVisual(m_Response, m_ResponseMeta);
                        }
                        m_ResponseVisual.CreateDrawing();
                    }
                }
                else
                {
                    if (m_RequestVisualFactory != null && m_RequestVisual.hasDrawing) m_RequestVisual.DeleteDrawing();
                    m_RequestVisual.hasDrawing = false;

                    if (m_ResponseVisualFactory != null && m_ResponseVisual.hasDrawing) m_ResponseVisual.DeleteDrawing();
                    m_ResponseVisual.hasDrawing = false;
                }
            }

            public void DrawWindow(int windowID, Rect windowRect)
            {
                GUI.Window(windowID, windowRect, DrawWindowContents, m_RequestMeta.Topic);
            }

            public IVisual GetVisual()
            {
                // TODO
                if (m_RequestVisual == null)
                {
                    if (m_RequestVisualFactory == null && m_Request != null) m_RequestVisualFactory = VisualFactoryRegistry.GetVisualizer(m_Request, m_RequestMeta);

                    if (m_RequestVisualFactory != null)
                    {
                        m_RequestVisual = m_RequestVisualFactory.CreateVisual(m_Request, m_RequestMeta);
                    }
                }

                return m_RequestVisual;
            }

            public void ClearVisual()
            {
                m_RequestVisual?.DeleteDrawing();
                m_RequestVisual = null;
                m_ResponseVisual?.DeleteDrawing();
                m_ResponseVisual = null;
            }

            public void SetRequest(Message request, MessageMetadata requestMeta)
            {
                m_Request = request;
                m_RequestMeta = requestMeta;
                m_RequestVisual.hasAction = false;
            }

            public void SetResponse(Message response, MessageMetadata responseMeta)
            {
                m_Response = response;
                m_ResponseMeta = responseMeta;
                m_ResponseVisual.hasAction = false;
            }

            void DrawWindowContents(int id)
            {
                m_State.WindowScrollPosition = GUILayout.BeginScrollView(m_State.WindowScrollPosition);

                if (m_Request == null)
                {
                    GUILayout.Label("Waiting for request...");
                }
                else
                {
                    if (m_RequestVisualFactory == null) m_RequestVisualFactory = VisualFactoryRegistry.GetVisualizer(m_Request, m_RequestMeta);

                    if (!m_RequestVisual.hasAction)
                    {
                        m_RequestVisual?.DeleteDrawing();
                        if (m_RequestVisual == null)
                        {
                            m_RequestVisual = m_RequestVisualFactory.CreateVisual(m_Response, m_ResponseMeta);
                        }
                    }

                    m_RequestVisual.OnGUI();
                }

                // horizontal line
                GUILayout.Label("", GUI.skin.horizontalSlider);

                if (m_Response == null)
                {
                    GUILayout.Label("Waiting for response...");
                }
                else
                {
                    if (m_ResponseVisualFactory == null) m_ResponseVisualFactory = VisualFactoryRegistry.GetVisualizer(m_Response, m_ResponseMeta);

                    if (!m_ResponseVisual.hasAction)
                    {
                        m_ResponseVisual?.DeleteDrawing();
                        if (m_ResponseVisual == null)
                        {
                            m_ResponseVisual = m_ResponseVisualFactory.CreateVisual(m_Response, m_ResponseMeta);
                        }
                    }

                    m_ResponseVisual.OnGUI();
                }

                GUILayout.EndScrollView();
            }
        }
    }
}
