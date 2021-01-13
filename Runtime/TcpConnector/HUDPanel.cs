using RosMessageGeneration;
using ROSGeometry;
using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

public class HUDPanel : MonoBehaviour
{
    [Tooltip("Assume ROS points are in this coordinate space")]
    public CoordinateSpaceSelection coordinateSpace = CoordinateSpaceSelection.FLU;
    public bool showPoints = true;

    // GUI variables
    GUIStyle labelStyle;
    GUIStyle contentStyle;
    GUIStyle messageStyle;
    GUIStyle sentStyle;
    GUIStyle recvStyle;
    bool viewSent = false;
    bool viewRecv = false;
    bool viewSrvs = false;
    Rect scrollRect;
    bool redrawGUI = false;
    Dictionary<string, Tuple<RosMessageTypes.Geometry.Point, SendRecv>> points;

    // ROS Message variables
    internal bool isEnabled;
    internal string host;

    MessageViewState lastMessageSent;
    string lastMessageSentMeta = "None";

    public void SetLastMessageSent(string topic, Message message)
    {
        lastMessageSent = new MessageViewState() { label = "Last Message Sent:", message = message, sendRecv = SendRecv.Recv };
        lastMessageSentMeta = $"{topic} (time: {System.DateTime.Now.TimeOfDay})";
        redrawGUI = true;
    }

    MessageViewState lastMessageReceived;
    string lastMessageReceivedMeta = "None";

    public void SetLastMessageReceived(string topic, Message message)
    {
        lastMessageReceived = new MessageViewState() { label = "Last Message Received:", message = message, sendRecv = SendRecv.Recv };
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
            normal = { textColor = Color.white },
            fontStyle = FontStyle.Bold,
            fixedWidth = 250
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

        sentStyle = new GUIStyle
        {
            alignment = TextAnchor.MiddleLeft,
            normal = { textColor = Color.yellow },
            fixedWidth = 300
        };

        recvStyle = new GUIStyle
        {
            alignment = TextAnchor.MiddleLeft,
            normal = { textColor = Color.red },
            fixedWidth = 300
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
        if(lastMessageSent != null)
            viewSent = GUILayout.Toggle(viewSent, "View contents");

        // Last message received
        GUILayout.Label("Last Message Received:", labelStyle);
        GUILayout.Label(lastMessageReceivedMeta, contentStyle);
        if (lastMessageReceived != null)
            viewRecv = GUILayout.Toggle(viewRecv, "View contents");

        GUILayout.Label($"{activeServices.Count} Active Service Requests:", labelStyle);
        if (activeServices.Count > 0)
        {
            var dots = new String('.', (int)Time.time % 4);
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
                y = ShowMessage(service, y, showElapsedTime:true);
            }

            if (lastCompletedServiceRequest != null && lastCompletedServiceResponse != null)
            {
                y = ShowMessage(lastCompletedServiceRequest, y);
                y = ShowMessage(lastCompletedServiceResponse, y);
            }
        }
    }

    enum SendRecv
    {
        Sent,
        Recv
    }

    class MessageViewState
    {
        public string label;
        public SendRecv sendRecv;
        public int serviceID;
        public float timestamp;
        public string topic;
        public Message message;
        public Rect contentRect;
        public Vector2 scrollPosition;
    }

    float ShowMessage(MessageViewState msgView, float y, bool showElapsedTime = false)
    {
        if (msgView == null)
            return y;

        // Show Points if applicable
        ParsePoint(msgView.message, msgView.sendRecv);

        // Start scrollviews
        msgView.scrollPosition = GUI.BeginScrollView(new Rect(0, y + 5, 325, 200), msgView.scrollPosition, msgView.contentRect);

        GUILayout.BeginVertical("box");

        // Paste contents of message
        if(showElapsedTime)
            GUILayout.Label($"{msgView.label} ({Time.time-msgView.timestamp})", labelStyle);
        else
            GUILayout.Label(msgView.label, labelStyle);
        GUILayout.Label(msgView.message.ToString(), messageStyle);

        GUILayout.EndVertical();
        GUI.EndScrollView();

        // Update size of internal rect view
        if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
            msgView.contentRect = GUILayoutUtility.GetLastRect();

        return msgView.contentRect.yMax;
    }

    /// <summary>
    /// Parse messages for Point or Vector data for display
    /// </summary>
    /// <param name="msg"></param>
    /// <param name="type"></param>
    private void ParsePoint(Message msg, SendRecv type)
    {
        if (msg == null || !showPoints)
            return;

        if (redrawGUI) // Only parse new data if a new message is received
        {
            var messageClass = msg.GetType();

            var fieldInfo = messageClass.GetFields();
            points = new Dictionary<string, Tuple<RosMessageTypes.Geometry.Point, SendRecv>>();

            // Iterate through fields to find point data
            foreach (var e in fieldInfo)
            {
                object obj = null;
                RosMessageTypes.Geometry.Point point = null;

                switch (e.FieldType.ToString())
                {
                    case "RosMessageTypes.Geometry.Pose":
                        obj = e.GetValue(msg);
                        point = ((RosMessageTypes.Geometry.Pose)obj).position;
                        break;
                    case "RosMessageTypes.Geometry.Point":
                        obj = e.GetValue(msg);
                        point = (RosMessageTypes.Geometry.Point)obj;
                        break;
                    default:
                        break;
                }

                // Display Point data
                if (point != null)
                {
                    points.Add(e.Name, Tuple.Create(point, type));
                }
            }
            redrawGUI = false;
        }

        // Convert and display all points
        foreach (var p in points)
        {
            var pt = p.Value.Item1;
            var sentType = p.Value.Item2;
            var screenPos = Camera.main.WorldToScreenPoint(pt.From(coordinateSpace));
            var convertedGUIPos = GUIUtility.ScreenToGUIPoint(screenPos);
            GUI.Label(new Rect(convertedGUIPos.x, Screen.height - convertedGUIPos.y, 0, 0), $"• {p.Key}", (sentType == SendRecv.Sent) ? sentStyle : recvStyle);
        }
    }
}