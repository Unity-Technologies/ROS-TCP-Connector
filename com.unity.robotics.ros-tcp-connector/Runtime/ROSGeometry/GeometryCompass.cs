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

        [SerializeField] CardinalDirection m_ZAxisDirection;

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
                if (!UnityEditor.AssetDatabase.IsValidFolder("Assets/Resources"))
                    UnityEditor.AssetDatabase.CreateFolder("Assets", "Resources");
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

    public static class GeometryCompassExtensions
    {
        public static Vector3 ToWorldDirection(this CardinalDirection desiredDirection)
        {
            switch (GeometryCompass.UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    switch (desiredDirection)
                    {
                        case CardinalDirection.North:
                            return new Vector3(0, 0, 1);
                        case CardinalDirection.East:
                            return new Vector3(1, 0, 0);
                        case CardinalDirection.South:
                            return new Vector3(0, 0, -1);
                        case CardinalDirection.West:
                            return new Vector3(-1, 0, 0);
                        default:
                            throw new Exception($"Unsupported CardinalDirection: {desiredDirection}");
                    }
                case CardinalDirection.East:
                    switch (desiredDirection)
                    {
                        case CardinalDirection.North:
                            return new Vector3(-1, 0, 0);
                        case CardinalDirection.East:
                            return new Vector3(0, 0, 1);
                        case CardinalDirection.South:
                            return new Vector3(1, 0, 0);
                        case CardinalDirection.West:
                            return new Vector3(0, 0, -1);
                        default:
                            throw new Exception($"Unsupported CardinalDirection: {desiredDirection}");
                    }
                case CardinalDirection.South:
                    switch (desiredDirection)
                    {
                        case CardinalDirection.North:
                            return new Vector3(0, 0, -1);
                        case CardinalDirection.East:
                            return new Vector3(-1, 0, 0);
                        case CardinalDirection.South:
                            return new Vector3(0, 0, 1);
                        case CardinalDirection.West:
                            return new Vector3(1, 0, 0);
                        default:
                            throw new Exception($"Unsupported CardinalDirection: {desiredDirection}");
                    }
                case CardinalDirection.West:
                    switch (desiredDirection)
                    {
                        case CardinalDirection.North:
                            return new Vector3(1, 0, 0);
                        case CardinalDirection.East:
                            return new Vector3(0, 0, -1);
                        case CardinalDirection.South:
                            return new Vector3(-1, 0, 0);
                        case CardinalDirection.West:
                            return new Vector3(0, 0, 1);
                        default:
                            throw new Exception($"Unsupported CardinalDirection: {desiredDirection}");
                    }
                default:
                    throw new Exception($"Unsupported CardinalDirection: {GeometryCompass.UnityZAxisDirection}");
            }
        }

        public static float ToUnityYawDegrees(this CardinalDirection desiredDirection)
        {
            switch (GeometryCompass.UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    switch (desiredDirection)
                    {
                        case CardinalDirection.North:
                            return 0.0f;
                        case CardinalDirection.East:
                            return 90.0f;
                        case CardinalDirection.South:
                            return 180.0f;
                        case CardinalDirection.West:
                            return -90.0f;
                        default:
                            throw new Exception($"Unsupported CardinalDirection: {desiredDirection}");
                    }
                case CardinalDirection.East:
                    switch (desiredDirection)
                    {
                        case CardinalDirection.North:
                            return -90.0f;
                        case CardinalDirection.East:
                            return 0.0f;
                        case CardinalDirection.South:
                            return 90.0f;
                        case CardinalDirection.West:
                            return 180.0f;
                        default:
                            throw new Exception($"Unsupported CardinalDirection: {desiredDirection}");
                    }
                case CardinalDirection.South:
                    switch (desiredDirection)
                    {
                        case CardinalDirection.North:
                            return 180.0f;
                        case CardinalDirection.East:
                            return -90.0f;
                        case CardinalDirection.South:
                            return 0.0f;
                        case CardinalDirection.West:
                            return 90.0f;
                        default:
                            throw new Exception($"Unsupported CardinalDirection: {desiredDirection}");
                    }
                case CardinalDirection.West:
                    switch (desiredDirection)
                    {
                        case CardinalDirection.North:
                            return 90.0f;
                        case CardinalDirection.East:
                            return 180.0f;
                        case CardinalDirection.South:
                            return -90.0f;
                        case CardinalDirection.West:
                            return 0.0f;
                        default:
                            throw new Exception($"Unsupported CardinalDirection: {desiredDirection}");
                    }
                default:
                    throw new Exception($"Unsupported CardinalDirection: {GeometryCompass.UnityZAxisDirection}");
            }
        }
    }
}
