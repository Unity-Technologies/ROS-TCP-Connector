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
        string rosIP = "127.0.0.1";
        string unityOverrideIP = "";

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
            rosIP = EditorGUILayout.TextField("ROS IP Address", rosIP);
            prefab.rosPort = EditorGUILayout.IntField("ROS Port", prefab.rosPort);
            EditorGUILayout.Space();
            unityOverrideIP = EditorGUILayout.TextField(
                new GUIContent("Override Unity IP Address", "If blank, determine IP automatically."),
                unityOverrideIP);
            prefab.unityPort = EditorGUILayout.IntField("Unity Port", prefab.unityPort);
            if (GUILayout.Button("Set IP"))
            {   
                if(unityOverrideIP != "")
                {
                    if(ROSConnection.IPFormatIsCorrect(unityOverrideIP))
                        prefab.overrideUnityIP = unityOverrideIP;
                    else
                        Debug.LogError("Override Unity IP Address is incorrect");
                }
                
                if(ROSConnection.IPFormatIsCorrect(rosIP))
                    prefab.rosIPAddress = rosIP;
                else
                    Debug.LogError("ROS IP Address is incorrect");
                
                this.Close();
            }
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("If awaiting a service response:", EditorStyles.boldLabel);
            prefab.awaitDataMaxRetries = EditorGUILayout.IntField(
                new GUIContent("Max Service Retries",
                    "While waiting for a service to respond, check this many times before giving up."),
                prefab.awaitDataMaxRetries);
            prefab.awaitDataSleepSeconds = EditorGUILayout.FloatField(
                new GUIContent("Sleep (seconds)",
                    "While waiting for a service to respond, wait this many seconds between checks."),
                prefab.awaitDataSleepSeconds);
            prefab.readChunkSize = EditorGUILayout.IntField(
                new GUIContent("Read chunk size",
                    "While reading received messages, read this many bytes at a time."),
                prefab.readChunkSize);
            prefab.awaitDataReadRetry = EditorGUILayout.IntField(
                new GUIContent("Max Read retries",
                    "While waiting to read a full message, check this many times before giving up."),
                prefab.awaitDataReadRetry);

            if (GUI.changed)
            {
                EditorUtility.SetDirty(prefabObj);
            }
        }
    }
}