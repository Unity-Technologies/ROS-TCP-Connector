using UnityEngine;

public class HUDPanel : MonoBehaviour
{
    GUIStyle labelStyle;
    GUIStyle contentStyle;
    GUIStyle messageStyle;
    bool viewSent = false;
    bool viewRecv = false;
    Vector2 scrollViewVector = Vector2.zero;
    Vector2 sentViewVector = Vector2.zero;
    Vector2 recvViewVector = Vector2.zero;
    Rect scrollRect;
    Rect sentRect;
    Rect recvRect;

    internal bool isEnabled;
    internal string host;
    internal string lastMessageSentMeta = "";
    internal string lastMessageSent = "";
    internal string lastMessageReceivedMeta = "";
    internal string lastMessageReceived = "";

    void Awake()
    {
        // Define font styles
        labelStyle = new GUIStyle
        {
            alignment = TextAnchor.MiddleLeft,
            normal = {textColor = Color.white},
            fontStyle = FontStyle.Bold
        };

        contentStyle = new GUIStyle
        {
            alignment = TextAnchor.MiddleLeft,
            padding = new RectOffset(10, 0, 0, 5),
            normal = {textColor = Color.white},
        };

        messageStyle = new GUIStyle
        {
            alignment = TextAnchor.MiddleLeft,
            padding = new RectOffset(10, 0, 5, 5),
            normal = {textColor = Color.white},
        };

        scrollRect = new Rect(0, 0, 335, 285);
        sentRect = new Rect(0, 0, 320, 200);
        recvRect = new Rect(0, 0, 320, 200);
    }

    void OnGUI() 
    {
        if (!isEnabled) return;

        // Initialize main HUD
        scrollViewVector = GUI.BeginScrollView(new Rect(0, 0, scrollRect.width + 15, 300), scrollViewVector, scrollRect);
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
        GUI.EndScrollView();

        // Update length of scroll
        if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
            scrollRect = GUILayoutUtility.GetLastRect();
        
        // Optionally show message contents
        if (viewSent)
            sentRect = ShowMessage("sent");

        if (viewRecv)
            recvRect = ShowMessage("recv");
    }

    private Rect ShowMessage(string type)
    {
        // Start scrollviews
        if (type == "sent")
            sentViewVector = GUI.BeginScrollView(new Rect(0, scrollRect.yMax + 5, sentRect.width + 20, 200), sentViewVector, sentRect);
        else
        {
            var offset = (viewSent) ? ((sentRect.height < 200) ? sentRect.yMax : scrollRect.yMax + 205) : scrollRect.yMax;
            recvViewVector = GUI.BeginScrollView(new Rect(0, offset + 5, recvRect.width + 20, 200), recvViewVector, recvRect);
        }
        GUILayout.BeginVertical("box");
        
        // Paste contents of message
        if (type == "sent")
        {
            GUILayout.Label($"Last Message Sent:", labelStyle);
            GUILayout.Label(lastMessageSent, messageStyle);
        }
        else
        {
            GUILayout.Label($"Last Message Received:", labelStyle);
            GUILayout.Label(lastMessageReceived, messageStyle);
        }
            
        
        GUILayout.EndVertical();
        GUI.EndScrollView();

        // Update size of internal rect view
        if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
            return GUILayoutUtility.GetLastRect();
        else 
            return (type == "sent") ? sentRect : recvRect;
    }
}