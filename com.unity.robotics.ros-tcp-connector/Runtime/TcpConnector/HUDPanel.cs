using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class HUDPanel : MonoBehaviour
    {
        // GUI variables
        GUIStyle m_LabelStyle;
        GUIStyle m_ContentStyle;
        GUIStyle m_MessageStyle;
        bool m_ViewSent = false;
        bool m_ViewRecv = false;
        bool m_ViewSrvs = false;
        Rect m_ScrollRect;

        // ROS Message variables
        internal bool isEnabled;
        internal string host;

        MessageViewState m_LastMessageSent;
        string m_LastMessageSentMeta = "None";
        float m_LastMessageSentRealtime;

        public void SetLastMessageSent(string topic, Message message)
        {
            m_LastMessageSent = new MessageViewState() { label = "Last Message Sent:", message = message };
            m_LastMessageSentMeta = $"{topic} (time: {System.DateTime.Now.TimeOfDay})";
            m_LastMessageSentRealtime = Time.realtimeSinceStartup;
        }

        MessageViewState m_LastMessageReceived;
        string m_LastMessageReceivedMeta = "None";
        float m_LastMessageReceivedRealtime;

        public void SetLastMessageReceived(string topic, Message message)
        {
            m_LastMessageReceived = new MessageViewState() { label = "Last Message Received:", message = message };
            m_LastMessageReceivedMeta = $"{topic} (time: {System.DateTime.Now.TimeOfDay})";
            m_LastMessageReceivedRealtime = Time.realtimeSinceStartup;
        }

        List<MessageViewState> activeServices = new List<MessageViewState>();
        MessageViewState lastCompletedServiceRequest = null;
        MessageViewState lastCompletedServiceResponse = null;
        int nextServiceID = 101;

        public int AddServiceRequest(string topic, Message request)
        {
            int serviceID = nextServiceID;
            nextServiceID++;

            activeServices.Add(new MessageViewState()
            {
                serviceID = serviceID,
                timestamp = Time.time,
                topic = topic,
                message = request,
                label = $"{topic} Service Requested",
            });

            return serviceID;
        }

        public void AddServiceResponse(int serviceID, Message response)
        {
            lastCompletedServiceRequest = activeServices.Find(s => s.serviceID == serviceID);
            activeServices.Remove(lastCompletedServiceRequest);

            lastCompletedServiceResponse = new MessageViewState()
            {
                serviceID = serviceID,
                timestamp = Time.time,
                topic = lastCompletedServiceRequest.topic,
                message = response,
                label = $"{lastCompletedServiceRequest.topic} Service Response",
            };
        }

        void Awake()
        {
            // Define font styles
            m_LabelStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
                fontStyle = FontStyle.Bold,
                fixedWidth = 250
            };

            m_ContentStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 0, 5),
                normal = { textColor = Color.white },
                fixedWidth = 300
            };

            m_MessageStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 5, 5),
                normal = { textColor = Color.white },
                fixedWidth = 300,
                wordWrap = true
            };

            m_ScrollRect = new Rect();
        }

        Color GetConnectionColor(float elapsedTime)
        {
            var bright = new Color(1, 1, 0.5f);
            var mid = new Color(0, 1, 1);
            var dark = new Color(0, 0.5f, 1);
            const float brightDuration = 0.03f;
            const float fadeToDarkDuration = 1.0f;

            if (!ROSConnection.instance.HasConnectionThread)
                return Color.gray;
            if (ROSConnection.instance.HasConnectionError)
                return Color.red;
            if (elapsedTime <= brightDuration)
                return bright;
            else
                return Color.Lerp(mid, dark, elapsedTime / fadeToDarkDuration);
        }

        void OnGUI()
        {
            if (!isEnabled)
                return;

            // Initialize main HUD
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(300));

            // ROS IP Setup
            GUILayout.BeginHorizontal();
            Color baseColor = GUI.color;
            GUI.color = GetConnectionColor(Time.realtimeSinceStartup - m_LastMessageReceivedRealtime);
            GUILayout.Label("\u25C0", m_LabelStyle, GUILayout.Width(10));
            GUI.color = GetConnectionColor(Time.realtimeSinceStartup - m_LastMessageSentRealtime);
            GUILayout.Label("\u25B6", m_LabelStyle, GUILayout.Width(15));
            GUI.color = baseColor;
            GUILayout.Label("ROS IP: ", m_LabelStyle, GUILayout.Width(100));

            if (!ROSConnection.instance.HasConnectionThread)
            {
                ROSConnection.instance.RosIPAddress = GUILayout.TextField(ROSConnection.instance.RosIPAddress);
                GUILayout.EndHorizontal();
                GUILayout.Label("(Not connected)");
                if (GUILayout.Button("Connect"))
                    ROSConnection.instance.Connect();
            }
            else
            {
                GUILayout.Label(host, m_ContentStyle);
                GUILayout.EndHorizontal();
            }

            // Last message sent
            GUILayout.Label("Last Message Sent:", m_LabelStyle);
            GUILayout.Label(m_LastMessageSentMeta, m_ContentStyle);
            if (m_LastMessageSent != null)
                m_ViewSent = GUILayout.Toggle(m_ViewSent, "View contents");

            // Last message received
            GUILayout.Label("Last Message Received:", m_LabelStyle);
            GUILayout.Label(m_LastMessageReceivedMeta, m_ContentStyle);
            if (m_LastMessageReceived != null)
                m_ViewRecv = GUILayout.Toggle(m_ViewRecv, "View contents");

            GUILayout.Label($"{activeServices.Count} Active Service Requests:", m_LabelStyle);
            if (activeServices.Count > 0)
            {
                var dots = new String('.', (int)Time.time % 4);
                GUILayout.Label($"Waiting for service response{dots}", m_ContentStyle);
            }

            m_ViewSrvs = GUILayout.Toggle(m_ViewSrvs, "View services status");

            GUILayout.EndVertical();

            // Update length of scroll
            if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
                m_ScrollRect = GUILayoutUtility.GetLastRect();

            // Optionally show message contents
            float y = m_ScrollRect.yMax;
            if (m_ViewSent)
            {
                y = ShowMessage(m_LastMessageSent, y);
            }

            if (m_ViewRecv)
            {
                y = ShowMessage(m_LastMessageReceived, y);
            }

            if (m_ViewSrvs)
            {
                foreach (MessageViewState service in activeServices)
                {
                    y = ShowMessage(service, y, showElapsedTime: true);
                }

                if (lastCompletedServiceRequest != null && lastCompletedServiceResponse != null)
                {
                    y = ShowMessage(lastCompletedServiceRequest, y);
                    y = ShowMessage(lastCompletedServiceResponse, y);
                }
            }
        }

        /// <summary>
        /// All the information necessary to display a message and remember its scroll position
        /// </summary>
        class MessageViewState
        {
            public string label;
            public int serviceID;
            public float timestamp;
            public string topic;
            public Message message;
            public Rect contentRect;
            public Vector2 scrollPosition;
        }

        /// <summary>
        /// Displays a MessageViewState
        /// </summary>
        /// <param name="msgView">The message view to draw</param>
        /// <param name="y">The Y position to draw at</param>
        /// <param name="showElapsedTime">Whether to add elapsed time to the title</param>
        /// <returns>The new Y position to draw at</returns>
        float ShowMessage(MessageViewState msgView, float y, bool showElapsedTime = false)
        {
            if (msgView == null)
                return y;

            // Start scrollviews
            float height = msgView.contentRect.height > 0 ? Mathf.Min(msgView.contentRect.height, 200) : 200;
            Rect panelRect = new Rect(0, y + 5, 325, height);
            msgView.scrollPosition = GUI.BeginScrollView(panelRect, msgView.scrollPosition, msgView.contentRect);

            GUILayout.BeginVertical("box");

            // Paste contents of message
            if (showElapsedTime)
                GUILayout.Label($"{msgView.label} ({Time.time - msgView.timestamp})", m_LabelStyle);
            else
                GUILayout.Label(msgView.label, m_LabelStyle);
            GUILayout.Label(msgView.message.ToString(), m_MessageStyle);

            GUILayout.EndVertical();
            GUI.EndScrollView();

            // Update size of internal rect view
            if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
                msgView.contentRect = GUILayoutUtility.GetLastRect();

            return panelRect.yMax;
        }
    }
}
