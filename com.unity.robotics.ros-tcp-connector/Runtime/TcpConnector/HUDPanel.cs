using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using Unity.Robotics.MessageVisualizers;

namespace Unity.Robotics.ROSTCPConnector
{
    public class HUDPanel : MonoBehaviour
    {
        // GUI variables
        public static GUIStyle headingStyle;
        public static GUIStyle ipStyle;
        public static GUIStyle contentStyle;
        public static GUIStyle messageStyle;
        public static GUIStyle boldStyle;
        public static GUIStyle boxStyle;
        Rect scrollRect;

        // ROS Message variables
        internal bool isEnabled;
        string rosConnectAddress = "";
        string unityListenAddress = "Not listening";

        MessageViewState lastMessageSent;
        MessageViewState lastMessageReceived;
        List<MessageViewState> activeServices = new List<MessageViewState>();
        MessageViewState lastCompletedServiceRequest;
        MessageViewState lastCompletedServiceResponse;
        int nextServiceID = 101;

        public void SetLastMessageSent(string topic, Message message)
        {
            bool foldedOut = false;
            if (lastMessageSent != null)
            {
                foldedOut = lastMessageSent.foldedOut;
                lastMessageSent.RemoveVisual();
            }
            lastMessageSent = new MessageViewState()
            {
                label = "Last Message Sent:",
                meta = new MessageMetadata(topic, DateTime.Now),
                message = message,
                foldedOut = foldedOut
            };
            lastMessageSent.InitVisualizer();
        }

        public void SetLastMessageReceived(string topic, Message message)
        {
            bool foldedOut = false;
            if (lastMessageReceived != null)
            {
                foldedOut = lastMessageSent.foldedOut;
                lastMessageReceived.RemoveVisual();
            }

            lastMessageReceived = new MessageViewState()
            {
                label = "Last Message Received:",
                meta = new MessageMetadata(topic, DateTime.Now),
                message = message,
                foldedOut = foldedOut
            };
            lastMessageReceived.InitVisualizer();
        }

        public int AddServiceRequest(string topic, Message request)
        {
            int serviceID = nextServiceID;
            nextServiceID++;

            MessageViewState newMsgView = new MessageViewState()
            {
                serviceID = serviceID,
                meta = new MessageMetadata(topic, DateTime.Now),
                message = request,
                label = "Active Request: ",
            };
            activeServices.Add(newMsgView);
            newMsgView.InitVisualizer();

            return serviceID;
        }

        public void AddServiceResponse(int serviceID, Message response)
        {
            bool requestFoldedOut = false;
            bool responseFoldedOut = false;
            if (lastCompletedServiceRequest != null)
            {
                requestFoldedOut = lastCompletedServiceRequest.foldedOut;
                lastCompletedServiceRequest.RemoveVisual();
            }

            if (lastCompletedServiceResponse != null)
            {
                responseFoldedOut = lastCompletedServiceResponse.foldedOut;
                lastCompletedServiceResponse.RemoveVisual();
            }

            lastCompletedServiceRequest = activeServices.Find(s => s.serviceID == serviceID);
            lastCompletedServiceRequest.label = "Last Completed Request: ";
            activeServices.Remove(lastCompletedServiceRequest);

            if (requestFoldedOut)
                lastCompletedServiceRequest.foldedOut = true;

            lastCompletedServiceResponse = new MessageViewState()
            {
                serviceID = serviceID,
                meta = new MessageMetadata(lastCompletedServiceRequest.meta.topic, DateTime.Now),
                message = response,
                label = "Last Completed Response: ",
                foldedOut = responseFoldedOut,
            };
            lastCompletedServiceResponse.InitVisualizer();
        }

        void Awake()
        {
            // Define font styles
            headingStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
                fontStyle = FontStyle.Bold,
                fixedWidth = 100
            };

            ipStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = Color.white },
            };

            boldStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 0, 5),
                normal = { textColor = Color.white },
                fontStyle = FontStyle.Bold,
                fixedWidth = 300
            };

            contentStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 0, 5),
                normal = { textColor = Color.white },
                fixedWidth = 300
            };

            messageStyle = new GUIStyle
            {
                alignment = TextAnchor.MiddleLeft,
                padding = new RectOffset(10, 0, 5, 5),
                normal = { textColor = Color.white },
                fixedWidth = 300,
                wordWrap = true
            };

            scrollRect = new Rect();
        }

        public void SetRosIP(string ip, int port)
        {
            rosConnectAddress = $"{ip}:{port}";
        }

        public void OnStartMessageServer(string ip, int port)
        {
            unityListenAddress = $"{ip}:{port}";
        }

        void OnGUI()
        {
            if (!isEnabled)
                return;

            if (boxStyle == null)
            {
                boxStyle = GUI.skin.GetStyle("box");
                boxStyle.fixedWidth = 300;
            }

            // Initialize main HUD
            GUILayout.BeginVertical(boxStyle);

            // ROS IP Setup
            GUILayout.BeginHorizontal();
            GUILayout.Label("ROS IP: ", headingStyle);
            GUILayout.Label(rosConnectAddress, ipStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Unity IP: ", headingStyle);
            GUILayout.Label(unityListenAddress, ipStyle);
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();

            // Update length of scroll
            if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
                scrollRect = GUILayoutUtility.GetLastRect();

            // Optionally show message contents
            float y = scrollRect.yMax;

            y = ShowMessage(lastMessageSent, y);
            y = ShowMessage(lastMessageReceived, y);

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

        /// <summary>
        /// All the information necessary to display a message and remember its scroll position
        /// </summary>
        class MessageViewState
        {
            public string label;
            public int serviceID;
            public bool foldedOut;
            public Message message;
            public MessageMetadata meta;
            public Rect contentRect;
            public Vector2 scrollPosition;
            public IVisualizerConfig visualizerConfig;
            public object visualizerDrawing;
            public Action visualizerGUI;

            public void OnGUI()
            {
                if (!foldedOut)
                    return;

                if (visualizerConfig == null)
                    visualizerConfig = MessageVisualizations.GetVisualizer(message, meta);

                if (visualizerGUI == null)
                    visualizerGUI = visualizerConfig.CreateGUI(message, meta, visualizerDrawing);

                visualizerGUI();
            }

            public void InitVisualizer()
            {
                //TODO: check whether the config wants gui and/or drawings
                CreateVisual();
            }

            public void CreateVisual()
            {
                if (visualizerDrawing != null)
                    return;

                if (visualizerConfig == null)
                    visualizerConfig = MessageVisualizations.GetVisualizer(message, meta);

                visualizerDrawing = visualizerConfig.CreateDrawing(message, meta);

                if (visualizerGUI != null)
                    visualizerGUI = null; // force GUI to be recreated with the new drawing
            }

            public void RemoveVisual()
            {
                if (visualizerDrawing == null)
                    return;

                visualizerConfig.DeleteDrawing(visualizerDrawing);
                visualizerDrawing = null;
            }
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

            GUILayout.BeginVertical(boxStyle);
            //GUILayout.Label(heading, labelStyle);
            string label = (showElapsedTime) ? $"{msgView.label} ({(DateTime.Now - msgView.meta.timestamp).TotalSeconds})" : msgView.label;
            GUILayout.Label(label, headingStyle);

            GUILayout.BeginHorizontal();
            bool newFoldedOut = GUILayout.Toggle(msgView.foldedOut, $"{msgView.meta.topic} {msgView.meta.timestamp.TimeOfDay}");
            msgView.foldedOut = newFoldedOut;
            GUILayout.EndHorizontal();

            // draw custom message visualization GUI
            msgView.OnGUI();

            GUILayout.EndVertical();
            GUI.EndScrollView();

            // Update size of internal rect view
            if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
                msgView.contentRect = GUILayoutUtility.GetLastRect();

            return panelRect.yMax;
        }
    }
}