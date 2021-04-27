using UnityEngine;
using UnityEditor;
using System.IO;

namespace Unity.Robotics.ROSTCPConnector.Editor
{
    public class ROSSettingsEditor : EditorWindow
    {
        [MenuItem("Robotics/ROS Settings", false, 0)]
        public static void OpenWindow()
        {
            ROSSettingsEditor window = GetWindow<ROSSettingsEditor>(false, "ROS Settings", true);
            window.minSize = new Vector2(300, 65);
            window.maxSize = new Vector2(600, 250);
            window.Show();
        }

        GameObject prefabObj;
        ROSConnection prefab;
        protected virtual void OnGUI()
        {
            if (prefab == null)
            {
                prefabObj = Resources.Load<GameObject>("ROSConnectionPrefab");
                if (prefabObj != null)
                    prefab = prefabObj.GetComponent<ROSConnection>();

                if (prefab == null)
                {
                    GameObject sceneObj = new GameObject("ROSConnection");
                    sceneObj.AddComponent<ROSConnection>();
                    if (!Directory.Exists("Assets/Resources"))
                        Directory.CreateDirectory("Assets/Resources");
                    prefabObj = PrefabUtility.SaveAsPrefabAsset(sceneObj,
                        "Assets/Resources/ROSConnectionPrefab.prefab");
                    if (prefabObj != null)
                        prefab = prefabObj.GetComponent<ROSConnection>();
                    DestroyImmediate(sceneObj);
                }
            }

            EditorGUILayout.LabelField("Settings for a new ROSConnection.instance", EditorStyles.boldLabel);
            prefab.rosIPAddress = EditorGUILayout.TextField("ROS IP Address", prefab.rosIPAddress);
            prefab.rosPort = EditorGUILayout.IntField("ROS Port", prefab.rosPort);
            EditorGUILayout.Space();

            if(!ROSConnection.IPFormatIsCorrect(prefab.rosIPAddress))
            {
                EditorGUILayout.HelpBox("ROS IP is invalid", MessageType.Warning);
            }
            EditorGUILayout.Space();

            prefab.keepaliveTime = EditorGUILayout.FloatField(
                new GUIContent("KeepAlive time (seconds)",
                    "Send a keepalive message this often. (The longer this time is, the longer it will take for ROSConnection to notice the Endpoint has stopped responding)."),
                prefab.keepaliveTime);

            if (GUI.changed)
            {
                EditorUtility.SetDirty(prefabObj);
            }
        }
    }
}