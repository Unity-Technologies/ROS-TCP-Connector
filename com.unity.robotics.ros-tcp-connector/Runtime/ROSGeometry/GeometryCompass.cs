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

        static Quaternion s_NinetyYaw = Quaternion.Euler(0, 90, 0);
        static Quaternion s_OneEightyYaw = Quaternion.Euler(0, 180, 0);
        static Quaternion s_NegativeNinetyYaw = Quaternion.Euler(0, -90, 0);

        public static Vector3<ENU> ToENU(Vector3 v)
        {
            switch (GetOrCreateSettings().m_ZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Vector3<ENU>(v.x, v.z, v.y);
                case CardinalDirection.East:
                    return new Vector3<ENU>(v.z, -v.x, v.y);
                case CardinalDirection.South:
                    return new Vector3<ENU>(-v.x, -v.z, v.y);
                case CardinalDirection.West:
                    return new Vector3<ENU>(-v.z, v.x, v.y);
                default:
                    throw new NotSupportedException();
            }
        }

        public static Quaternion<ENU> ToENU(Quaternion q)
        {
            switch (GetOrCreateSettings().m_ZAxisDirection)
            {
                case CardinalDirection.North:
                    q = s_NegativeNinetyYaw * q;
                    break;
                case CardinalDirection.East:
                    break;
                case CardinalDirection.South:
                    q = s_NinetyYaw * q;
                    break;
                case CardinalDirection.West:
                    q = s_OneEightyYaw * q;
                    break;
                default:
                    throw new NotSupportedException();
            }

            var r = q.To<FLU>();
            return new Quaternion<ENU>(r.x, r.y, r.z, r.w);
        }

        public static Vector3 FromENU(Vector3<ENU> v)
        {
            switch (GetOrCreateSettings().m_ZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Vector3(v.x, v.z, v.y);
                case CardinalDirection.East:
                    return new Vector3(-v.y, v.z, v.x);
                case CardinalDirection.South:
                    return new Vector3(-v.x, v.z, -v.y);
                case CardinalDirection.West:
                    return new Vector3(v.y, v.z, -v.x);
                default:
                    throw new NotSupportedException();
            }
        }

        public static Quaternion FromENU(Quaternion<ENU> q)
        {
            var r = new Quaternion(-q.y, q.z, q.x, -q.w);
            switch (GetOrCreateSettings().m_ZAxisDirection)
            {
                case CardinalDirection.North:
                    return s_NinetyYaw * r;
                case CardinalDirection.East:
                    return r;
                case CardinalDirection.South:
                    return s_NegativeNinetyYaw * r;
                case CardinalDirection.West:
                    return s_OneEightyYaw * r;
                default:
                    throw new NotSupportedException();
            }
        }

        public static Vector3<NED> ToNED(Vector3 v)
        {
            switch (GetOrCreateSettings().m_ZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Vector3<NED>(v.z, v.x, -v.y);
                case CardinalDirection.East:
                    return new Vector3<NED>(-v.x, v.z, -v.y);
                case CardinalDirection.South:
                    return new Vector3<NED>(-v.z, -v.x, -v.y);
                case CardinalDirection.West:
                    return new Vector3<NED>(v.x, -v.z, -v.y);
                default:
                    throw new NotSupportedException();
            }
        }

        public static Quaternion<NED> ToNED(Quaternion q)
        {
            switch (GetOrCreateSettings().m_ZAxisDirection)
            {
                case CardinalDirection.North:
                    break;
                case CardinalDirection.East:
                    q = s_NinetyYaw * q;
                    break;
                case CardinalDirection.South:
                    q = s_OneEightyYaw * q;
                    break;
                case CardinalDirection.West:
                    q = s_NegativeNinetyYaw * q;
                    break;
                default:
                    throw new NotSupportedException();
            }

            var r = q.To<FRD>();
            return new Quaternion<NED>(r.x, r.y, r.z, r.w);
        }

        public static Vector3 FromNED(Vector3<NED> v)
        {
            switch (GetOrCreateSettings().m_ZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Vector3(v.y, -v.z, v.x);
                case CardinalDirection.East:
                    return new Vector3(-v.x, -v.z, v.y);
                case CardinalDirection.South:
                    return new Vector3(-v.y, -v.z, -v.x);
                case CardinalDirection.West:
                    return new Vector3(v.x, -v.z, -v.y);
                default:
                    throw new NotSupportedException();
            }
        }

        public static Quaternion FromNED(Quaternion<NED> q)
        {
            var r = new Quaternion(q.y, -q.z, q.x, -q.w);
            switch (GetOrCreateSettings().m_ZAxisDirection)
            {
                case CardinalDirection.North:
                    return r;
                case CardinalDirection.East:
                    return s_NegativeNinetyYaw * r;
                case CardinalDirection.South:
                    return s_OneEightyYaw * r;
                case CardinalDirection.West:
                    return s_NinetyYaw * r;
                default:
                    throw new NotSupportedException();
            }
        }
    }
}
