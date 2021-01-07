using UnityEngine;

public class HUDPanel : MonoBehaviour
{
    GUIStyle labelStyle;
    GUIStyle contentStyle;
    
    internal bool isEnabled;
    internal string host;
    internal string lastMessageSentMeta = "";
    internal string lastMessageSent = "";
    internal string lastMessageReceivedMeta = "";
    internal string lastMessageReceived = "";

    void Awake()
    {
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
    }

    void OnGUI() 
    {
        if (!isEnabled) return;

        GUILayout.BeginVertical("box");
        GUILayout.Label($"ROS IP:", labelStyle);
        GUILayout.Label(host, contentStyle);
        GUILayout.Label($"Last Message Sent:", labelStyle);
        GUILayout.Label(lastMessageSentMeta, contentStyle);
        GUILayout.Label(lastMessageSent, contentStyle);
        GUILayout.Label($"Last Message Received:", labelStyle);
        GUILayout.Label(lastMessageReceivedMeta, contentStyle);
        GUILayout.Label(lastMessageReceived, contentStyle);
        GUILayout.EndVertical();
    }
}