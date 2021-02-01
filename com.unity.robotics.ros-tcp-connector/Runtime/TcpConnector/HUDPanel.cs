using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class HUDPanel : MonoBehaviour
    {
        // GUI variables
        GUIStyle labelStyle;
        GUIStyle contentStyle;
        GUIStyle messageStyle;
        bool viewSent = false;
        bool viewRecv = false;
        bool viewSrvs = false;
        Rect scrollRect;
        bool redrawGUI = false;

        // ROS Message variables
        internal bool isEnabled;
        internal string host;

        MessageViewState lastMessageSent;
        string lastMessageSentMeta = "None";

        public void SetLastMessageSent(string topic, Message message)
        {
            lastMessageSent = new MessageViewState() {label = "Last Message Sent:", message = message};
            lastMessageSentMeta = $"{topic} (time: {System.DateTime.Now.TimeOfDay})";
            redrawGUI = true;
        }

        MessageViewState lastMessageReceived;
        string lastMessageReceivedMeta = "None";

        public void SetLastMessageReceived(string topic, Message message)
        {
            lastMessageReceived = new MessageViewState() {label = "Last Message Received:", message = message};
            lastMessageReceivedMeta = $"{topic} (time: {System.DateTime.Now.TimeOfDay})";
            redrawGUI = true;
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
            labelStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = {textColor = Color.white},
                fontStyle = FontStyle.Bold,
                fixedWidth = 250
            };

            contentStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 0, 5),
                normal = {textColor = Color.white},
                fixedWidth = 300
            };

            messageStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 5, 5),
                normal = {textColor = Color.white},
                fixedWidth = 300,
                wordWrap = true
            };

            scrollRect = new Rect();
        }

        void OnGUI()
        {
            if (!isEnabled)
                return;

            // Initialize main HUD
            GUILayout.BeginVertical("box");

            // ROS IP Setup
            GUILayout.Label("ROS IP:", labelStyle);
            GUILayout.Label(host, contentStyle);

            // Last message sent
            GUILayout.Label("Last Message Sent:", labelStyle);
            GUILayout.Label(lastMessageSentMeta, contentStyle);
            if (lastMessageSent != null)
                viewSent = GUILayout.Toggle(viewSent, "View contents");

            // Last message received
            GUILayout.Label("Last Message Received:", labelStyle);
            GUILayout.Label(lastMessageReceivedMeta, contentStyle);
            if (lastMessageReceived != null)
                viewRecv = GUILayout.Toggle(viewRecv, "View contents");

            GUILayout.Label($"{activeServices.Count} Active Service Requests:", labelStyle);
            if (activeServices.Count > 0)
            {
                var dots = new String('.', (int) Time.time % 4);
                GUILayout.Label($"Waiting for service response{dots}", contentStyle);
            }

            viewSrvs = GUILayout.Toggle(viewSrvs, "View services status");

            GUILayout.EndVertical();

            // Update length of scroll
            if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
                scrollRect = GUILayoutUtility.GetLastRect();

            // Optionally show message contents
            float y = scrollRect.yMax;
            if (viewSent)
            {
                y = ShowMessage(lastMessageSent, y);
            }

            if (viewRecv)
            {
                y = ShowMessage(lastMessageReceived, y);
            }

            if (viewSrvs)
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
                GUILayout.Label($"{msgView.label} ({Time.time - msgView.timestamp})", labelStyle);
            else
                GUILayout.Label(msgView.label, labelStyle);
            GUILayout.Label(msgView.message.ToString(), messageStyle);

            GUILayout.EndVertical();
            GUI.EndScrollView();

            // Update size of internal rect view
            if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
                msgView.contentRect = GUILayoutUtility.GetLastRect();

            return panelRect.yMax;
        }
    }
}