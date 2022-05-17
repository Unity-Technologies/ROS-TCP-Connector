using UnityEngine;
using UnityEditor;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEditor.Callbacks;

namespace Unity.Robotics.ROSTCPConnector.Editor
{
    public class ROSSettingsEditor : EditorWindow
    {
        [MenuItem("Robotics/ROS Settings", false, 0)]
        public static void OpenWindow()
        {
            ROSSettingsEditor window = GetWindow<ROSSettingsEditor>(false, "ROS Settings", true);
            window.minSize = new Vector2(300, 65);
            window.maxSize = new Vector2(600, 500);
            window.Show();
        }

        GameObject prefabObj;
        Unity.Robotics.ROSTCPConnector.ROSConnection prefab;
        GeometryCompass geometryCompassPrefab;

        public enum RosProtocol
        {
            ROS1,
            ROS2
        }

#if ROS2
        RosProtocol m_SelectedProtocol = RosProtocol.ROS2;
        const RosProtocol k_AlternateProtocol = RosProtocol.ROS1;
#else
        RosProtocol m_SelectedProtocol = RosProtocol.ROS1;
        const RosProtocol k_AlternateProtocol = RosProtocol.ROS2;
#endif

        [SerializeField] string[] m_TFTopics;
        UnityEditor.Editor editor;

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
                    prefabObj = PrefabUtility.SaveAsPrefabAsset(sceneObj, "Assets/Resources/ROSConnectionPrefab.prefab");
                    if (prefabObj != null)
                        prefab = prefabObj.GetComponent<ROSConnection>();
                    DestroyImmediate(sceneObj);
                }
            }

            prefab.ConnectOnStart = EditorGUILayout.Toggle("Connect on Startup", prefab.ConnectOnStart);

            if (m_SelectedProtocol == k_AlternateProtocol)
            {
                EditorGUI.BeginDisabledGroup(true);
                EditorGUILayout.EnumPopup("Protocol", m_SelectedProtocol);
                EditorGUILayout.LabelField("(Recompiling, please wait...)");
                EditorGUI.EndDisabledGroup();
            }
            else
            {
                m_SelectedProtocol = (RosProtocol)EditorGUILayout.EnumPopup("Protocol", m_SelectedProtocol);
                if (m_SelectedProtocol == k_AlternateProtocol)
                {
                    var buildTarget = EditorUserBuildSettings.activeBuildTarget;
                    var buildTargetGroup = BuildPipeline.GetBuildTargetGroup(buildTarget);

                    List<string> allDefines = PlayerSettings.GetScriptingDefineSymbolsForGroup(buildTargetGroup).Split(';').ToList();
                    if (m_SelectedProtocol == RosProtocol.ROS1)
                    {
                        allDefines.Remove("ROS2");
                        Debug.Log($"Removing 'ROS2' from the scripting define symbols for build target '{buildTargetGroup}'.");
                    }
                    else
                    {
                        allDefines.Add("ROS2");
                        Debug.Log($"Adding 'ROS2' to the scripting define symbols for build target '{buildTargetGroup}'.");
                    }

                    PlayerSettings.SetScriptingDefineSymbolsForGroup(buildTargetGroup, string.Join(";", allDefines));
                }
            }

            EditorGUILayout.LabelField("Settings for a new ROSConnection.instance", EditorStyles.boldLabel);

            prefab.RosIPAddress = EditorGUILayout.TextField("ROS IP Address", prefab.RosIPAddress);
            prefab.RosPort = EditorGUILayout.IntField("ROS Port", prefab.RosPort);

            // Also set the player prefs, for users who hit play in the editor: they will expect the last-used IP address to appear in the hud
            ROSConnection.SetIPPref(prefab.RosIPAddress);
            ROSConnection.SetPortPref(prefab.RosPort);

            EditorGUILayout.Space();

            if (!ROSConnection.IPFormatIsCorrect(prefab.RosIPAddress))
            {
                EditorGUILayout.HelpBox("ROS IP is invalid", MessageType.Warning);
            }

            EditorGUILayout.Space();

            prefab.ShowHud = EditorGUILayout.Toggle("Show HUD", prefab.ShowHud);

            EditorGUILayout.Space();

            prefab.KeepaliveTime = EditorGUILayout.FloatField(
                new GUIContent("KeepAlive time (secs)",
                    "If no other messages are being sent, test the connection this often. (The longer this time is, the longer it will take for ROSConnection to notice the Endpoint has stopped responding)."),
                prefab.KeepaliveTime);

            prefab.NetworkTimeoutSeconds = EditorGUILayout.FloatField(
                new GUIContent("Network timeout (secs)",
                    "If a network message takes this long to send, assume the connection has failed. (The longer this time is, the longer it will take for ROSConnection to notice the Endpoint has stopped responding)."),
                prefab.NetworkTimeoutSeconds);

            prefab.SleepTimeSeconds = EditorGUILayout.FloatField(
                new GUIContent("Sleep time (secs)",
                    "Sleep this long before checking for new network messages. (Decreasing this time will make it respond faster, but consume more CPU)."),
                prefab.SleepTimeSeconds);

            EditorGUILayout.Space();

            prefab.listenForTFMessages = EditorGUILayout.Toggle("Listen for TF Messages", prefab.listenForTFMessages);

            // TODO: make the settings input update the prefab
            // EditorGUILayout.Space();
            // if (!editor) { editor = UnityEditor.Editor.CreateEditor(this); }
            // if (editor) { editor.OnInspectorGUI(); }

            if (GUI.changed)
            {
                PrefabUtility.SavePrefabAsset(prefab.gameObject);
            }

            // ROS Geometry Compass Settings
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Compass settings", EditorStyles.boldLabel);
            GeometryCompass.UnityZAxisDirection = (CardinalDirection)EditorGUILayout.EnumPopup("Unity Z Axis Direction", GeometryCompass.UnityZAxisDirection);
        }

        void OnInspectorUpdate() { Repaint(); }
    }

    [CustomEditor(typeof(ROSSettingsEditor), true)]
    public class TFTopicsEditorDrawer : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            var list = serializedObject.FindProperty("m_TFTopics");
            EditorGUILayout.PropertyField(list, new GUIContent("TF Topics"), true);
        }
    }
}
