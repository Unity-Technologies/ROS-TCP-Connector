using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Analytics;

public static class RosEditorAnalytics
{
    static bool s_EventRegistered = false;
    const int k_MaxEventsPerHour = 1000;
    const int k_MaxNumberOfElements = 1000;
    const string k_VendorKey = "unity.robotics";
    const string k_EventName = "RoboticsUsage";

    static bool EnableAnalytics()
    {
        AnalyticsResult result = EditorAnalytics.RegisterEventWithLimit(k_EventName, k_MaxEventsPerHour, k_MaxNumberOfElements, k_VendorKey);
        if (result == AnalyticsResult.Ok)
            s_EventRegistered = true;

        return s_EventRegistered;
    }

    struct AnalyticsData
    {
        public string protocol;
    }

    public static void SendAnalytics()
    {
        if (!s_EventRegistered)
            return;

        var data = new AnalyticsData()
        {
#if ROS2
            protocol = "ROS2",
#else
            protocol = "ROS1",
#endif
        };

        EditorAnalytics.SendEventWithLimit(k_EventName, data);
    }

    [InitializeOnLoadMethod]
    public static void InitializerMethod()
    {
        EnableAnalytics();
        SendAnalytics();
    }
}
