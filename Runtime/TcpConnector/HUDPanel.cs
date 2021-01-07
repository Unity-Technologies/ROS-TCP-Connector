using UnityEngine;

public class HUDPanel : MonoBehaviour
{
    GUIStyle labelStyle;
    GUIStyle contentStyle;
    bool viewSent = false;
    bool viewRecv = false;
    Vector2 scrollViewVector = Vector2.zero;
    Rect scrollRect;
    
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

        scrollRect = new Rect(0, 0, 335, 285);
    }

    void OnGUI() 
    {
        if (!isEnabled) return;

        scrollViewVector = GUI.BeginScrollView(new Rect(0, 0, scrollRect.width + 15, 300), scrollViewVector, scrollRect);
        GUILayout.BeginVertical("box");
        
        // ROS IP Setup
        GUILayout.Label($"ROS IP:", labelStyle);
        GUILayout.Label(host, contentStyle);

        // Last message sent
        GUILayout.Label($"Last Message Sent:", labelStyle);
        GUILayout.Label(lastMessageSentMeta, contentStyle);
        viewSent = GUILayout.Toggle(viewSent, "View contents");
        if (viewSent)
            GUILayout.Label(lastMessageSent, contentStyle);

        // Last message received
        GUILayout.Label($"Last Message Received:", labelStyle);
        GUILayout.Label(lastMessageReceivedMeta, contentStyle);
        viewRecv = GUILayout.Toggle(viewRecv, "View contents");
        if (viewRecv)
            GUILayout.Label(lastMessageReceived, contentStyle);
            
        GUILayout.EndVertical();
        GUI.EndScrollView();

        if (GUILayoutUtility.GetLastRect().height > 1 && GUILayoutUtility.GetLastRect().width > 1)
            scrollRect = GUILayoutUtility.GetLastRect();
    }
}