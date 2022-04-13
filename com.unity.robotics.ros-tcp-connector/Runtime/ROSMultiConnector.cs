using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ROSMultiConnector
{
    private static Dictionary<string, ROSConnection> _dict;
    public static ROSConnection GetConnectionInstance(string parentName, int portNum)
    {
        if (_dict == null)
            _dict = new Dictionary<string, ROSConnection>();
        if (_dict.ContainsKey(parentName))
            return _dict[parentName];

        GameObject prefab = Resources.Load<GameObject>("ROSConnectionPrefab");
        ROSConnection _instance;
        if (prefab == null)
        {
            Debug.LogWarning(
                "No settings for ROSConnection.instance! Open \"ROS Settings\" from the Robotics menu to configure it.");
            GameObject instance = new GameObject("ROSConnection");
            _instance = instance.AddComponent<ROSConnection>();
        }
        else
        {
            _instance = Object.Instantiate(prefab).GetComponent<ROSConnection>();
        }

        if (portNum != -1)
        {
            _instance.RosPort = portNum;
        }

        _instance.robotName = parentName;

        _dict.Add(parentName, _instance);

        return _dict[parentName];
    }

    public static ROSConnection GetConnectionInstanceByParentName(MonoBehaviour callingObj, int portNum = -1, string robotName = "")
    {
        var parent = string.IsNullOrEmpty(robotName) ? GetParentName(callingObj.gameObject) : robotName;
        return GetConnectionInstance(parent, portNum);
    }

    public static string GetParentName(GameObject currentObj)
    {
        var parent = currentObj.transform.parent;
        var prev = parent;
        while (parent != null)
        {
            parent = parent.transform.parent;
            if (parent != null)
                prev = parent;
        }
        return prev == null ? currentObj.gameObject.name : prev.gameObject.name;

    }
}
