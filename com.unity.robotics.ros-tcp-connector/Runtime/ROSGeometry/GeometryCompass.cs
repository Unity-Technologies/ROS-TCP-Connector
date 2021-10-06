using System;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.ROSGeometry
{
    public enum CardinalDirection
    {
        North = 0,
        East = 1,
        South = 2,
        West = 3,
    }

    public class GeometryCompass : ScriptableObject
    {
        public const string k_CompassSettingsAsset = "GeometryCompassSettings.asset";

        [SerializeField]
        CardinalDirection m_ZAxisDirection;

        static GeometryCompass s_Instance;

        static GeometryCompass GetOrCreateSettings()
        {
            if (s_Instance != null)
                return s_Instance;
#if UNITY_EDITOR
            string assetPath = System.IO.Path.Combine("Assets/Resources", k_CompassSettingsAsset);
            s_Instance = UnityEditor.AssetDatabase.LoadAssetAtPath<GeometryCompass>(assetPath);
            if (s_Instance == null)
            {
                s_Instance = ScriptableObject.CreateInstance<GeometryCompass>();
                s_Instance.m_ZAxisDirection = CardinalDirection.North;
                UnityEditor.AssetDatabase.CreateAsset(s_Instance, assetPath);
                UnityEditor.AssetDatabase.SaveAssets();
            }
#else
            s_Instance = Resources.Load<GeometryCompass>(k_CompassSettingsAsset);
#endif
            return s_Instance;
        }

        public static CardinalDirection UnityZAxisDirection
        {
            get
            {
                GeometryCompass compass = GetOrCreateSettings();
                return (compass == null) ? CardinalDirection.North : compass.m_ZAxisDirection;
            }

#if UNITY_EDITOR
            set
            {
                GeometryCompass compass = GetOrCreateSettings();
                if (compass.m_ZAxisDirection != value)
                {
                    compass.m_ZAxisDirection = value;
                    UnityEditor.EditorUtility.SetDirty(compass);
                    UnityEditor.AssetDatabase.SaveAssets();
                }
            }
#endif
        }

        public static readonly Quaternion k_NinetyYaw = Quaternion.Euler(0, 90, 0);
        public static readonly Quaternion k_OneEightyYaw = Quaternion.Euler(0, 180, 0);
        public static readonly Quaternion k_NegativeNinetyYaw = Quaternion.Euler(0, -90, 0);
    }
}
