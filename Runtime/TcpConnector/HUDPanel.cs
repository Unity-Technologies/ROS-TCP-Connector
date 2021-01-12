using RosMessageGeneration;
using ROSGeometry;
using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

public class HUDPanel : MonoBehaviour
{
    [Tooltip("Convert points to/from FLU for visualization?")]
    public bool convertPoints = true;
    
    // GUI variables
    GUIStyle labelStyle;
    GUIStyle contentStyle;
    GUIStyle messageStyle;
    GUIStyle sentStyle;
    GUIStyle recvStyle;
    bool viewSent = false;
    bool viewRecv = false;
    Vector2 sentViewVector = Vector2.zero;
    Vector2 recvViewVector = Vector2.zero;
    Rect scrollRect;
    Rect sentRect;
    Rect recvRect;
    bool redrawGUI = false;
    Dictionary<string, Tuple<RosMessageTypes.Geometry.Point, string>> points;

    // ROS Message variables
    internal bool isEnabled;
    internal string host;
    internal string lastMessageSentMeta = "";
    internal Message _lastMessageSent;
    internal string lastMessageReceivedMeta = "";
    internal Message _lastMessageReceived;

    public Message lastMessageSent
    {
        get { return _lastMessageSent; }
        set
        {
            _lastMessageSent = value;
            redrawGUI = true;
        }
    }

    public Message lastMessageReceived
    {
        get { return _lastMessageReceived; }
        set
        {
            _lastMessageReceived = value;
            redrawGUI = true;
        }
    }

    void Awake()
    {
        // Define font styles
        labelStyle = new GUIStyle
        {
            alignment = TextAnchor.MiddleLeft,
            normal = {textColor = Color.white},
            fontStyle = FontStyle.Bold,
            fixedWidth = 300
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

        sentStyle = new GUIStyle
        {
            alignment = TextAnchor.MiddleLeft,
            normal = {textColor = Color.yellow},
            fixedWidth = 300
        };

        recvStyle = new GUIStyle
        {
            alignment = TextAnchor.MiddleLeft,
            normal = {textColor = Color.red},
            fixedWidth = 300
        };

        scrollRect = new Rect();
        sentRect = new Rect();
        recvRect = new Rect();
    }

    void OnGUI() 
    {
        if (!isEnabled) return;

        // Initialize main HUD
        GUILayout.BeginVertical("box");
        
        // ROS IP Setup
        GUILayout.Label($"ROS IP:", labelStyle);
        GUILayout.Label(host, contentStyle);

        // Last message sent
        GUILayout.Label($"Last Message Sent:", labelStyle);
        GUILayout.Label(lastMessageSentMeta, contentStyle);
        viewSent = GUILayout.Toggle(viewSent, "View contents");

        // Last message received
        GUILayout.Label($"Last Message Received:", labelStyle);
        GUILayout.Label(lastMessageReceivedMeta, contentStyle);
        viewRecv = GUILayout.Toggle(viewRecv, "View contents");
            
        GUILayout.EndVertical();

        // Update length of scroll
        if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
            scrollRect = GUILayoutUtility.GetLastRect();
        
        // Optionally show message contents
        if (viewSent)
            sentRect = ShowMessage("sent");

        if (viewRecv)
            recvRect = ShowMessage("recv");
    }

    /// <summary>
    /// Display message contents as string.
    /// </summary>
    /// <param name="type"></param>
    /// <returns></returns>
    private Rect ShowMessage(string type)
    {
        // Show Points if applicable
        ParsePoint((type == "sent") ? lastMessageSent : lastMessageReceived, type);

        // Start scrollviews
        if (type == "sent")
            sentViewVector = GUI.BeginScrollView(new Rect(0, scrollRect.yMax + 5, 325, 200), sentViewVector, sentRect);
        else
        {
            var offset = (viewSent) ? ((sentRect.height < 200) ? sentRect.yMax : scrollRect.yMax + 205) : scrollRect.yMax;
            recvViewVector = GUI.BeginScrollView(new Rect(0, offset + 5, 325, 200), recvViewVector, recvRect);
        }
        GUILayout.BeginVertical("box");
        
        // Paste contents of message
        if (type == "sent")
        {
            GUILayout.Label($"Last Message Sent:", labelStyle);
            GUILayout.Label(lastMessageSent.ToString(), messageStyle);
        }
        else
        {
            GUILayout.Label($"Last Message Received:", labelStyle);
            GUILayout.Label(lastMessageReceived.ToString(), messageStyle);
        }
            
        GUILayout.EndVertical();
        GUI.EndScrollView();

        // Update size of internal rect view
        if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
            return GUILayoutUtility.GetLastRect();
        else 
            return (type == "sent") ? sentRect : recvRect;
    }

    /// <summary>
    /// Parse messages for Point or Vector data for display
    /// </summary>
    /// <param name="msg"></param>
    /// <param name="type"></param>
    private void ParsePoint(Message msg, string type)
    {
        if (msg == null) return;

        if (redrawGUI)
        {
            var messageClass = msg.GetType();
            var fieldInfo = messageClass.GetFields();
            points = new Dictionary<string, Tuple<RosMessageTypes.Geometry.Point, string>>();

            // Iterate through fields to find point data
            foreach (var e in fieldInfo)
            {
                object obj = null;
                RosMessageTypes.Geometry.Point point = null;

                switch(e.FieldType.ToString()) 
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
        }

        // Display all points
        foreach (var p in points)
        {
            var pt = p.Value.Item1;
            var sentType = p.Value.Item2;
            var screenPos = convertPoints ? Camera.main.WorldToScreenPoint(pt.From<FLU>()) : Camera.main.WorldToScreenPoint(new Vector3((float)pt.x, (float)pt.y, (float)pt.z));
            var convertedGUIPos = GUIUtility.ScreenToGUIPoint(screenPos);
            GUI.Label(new Rect(convertedGUIPos.x, Screen.height - convertedGUIPos.y, 0, 0), $"• {p.Key}", (sentType == "sent") ? sentStyle : recvStyle);
        }
        redrawGUI = false;
    }
}