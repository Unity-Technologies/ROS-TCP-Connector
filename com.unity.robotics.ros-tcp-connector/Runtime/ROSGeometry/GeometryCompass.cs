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

    public class GeometryCompass : MonoBehaviour
    {
        static GeometryCompass s_Instance;
        public static GeometryCompass Instance
        {
            get
            {
                if (s_Instance == null)
                {
                    var prefab = Resources.Load<GameObject>(PrefabName);
                    if (prefab == null)
                    {
                        Debug.LogWarning("No settings for GeometryCompass.instance! " +
                                         "Open \"ROS Settings\" from the Robotics menu to configure it.");
                        s_Instance = new GameObject("GeometryCompass").AddComponent<GeometryCompass>();
                    }
                    else
                    {
                        s_Instance = Instantiate(prefab).GetComponent<GeometryCompass>();
                    }
                }
                return s_Instance;
            }
        }

        public CardinalDirection UnityZAxisDirection = CardinalDirection.East;

        public const string PrefabName = "CompassGeometryPrefab";

        private static Quaternion s_NinetyYaw = Quaternion.Euler(0, 90, 0);
        private static Quaternion s_OneEightyYaw = Quaternion.Euler(0, 180, 0);
        private static Quaternion s_NegativeNinetyYaw = Quaternion.Euler(0, -90, 0);

        public Vector3<ENUGlobal> ToENU(Vector3 v)
        {
            switch (UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Vector3<ENUGlobal>(v.x, v.z, v.y);
                case CardinalDirection.East:
                    return new Vector3<ENUGlobal>(v.z, -v.x, v.y);
                case CardinalDirection.South:
                    return new Vector3<ENUGlobal>(-v.x, -v.z, v.y);
                case CardinalDirection.West:
                    return new Vector3<ENUGlobal>(-v.z, v.x, v.y);
                default:
                    throw new NotSupportedException();
            }
        }

        public Quaternion<ENUGlobal> ToENU(Quaternion q)
        {
            switch (UnityZAxisDirection)
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
            return new Quaternion<ENUGlobal>(r.x, r.y, r.z, r.w);
        }

        public Vector3 FromENU(Vector3<ENUGlobal> v)
        {
            switch (UnityZAxisDirection)
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

        public Quaternion FromENU(Quaternion<ENUGlobal> q)
        {
            var r = new Quaternion(-q.y, q.z, q.x, -q.w);
            switch (UnityZAxisDirection)
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

        public Vector3<NEDGlobal> ToNED(Vector3 v)
        {
            switch (UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Vector3<NEDGlobal>(v.z, v.x, -v.y);
                case CardinalDirection.East:
                    return new Vector3<NEDGlobal>(-v.x, v.z, -v.y);
                case CardinalDirection.South:
                    return new Vector3<NEDGlobal>(-v.z, -v.x, -v.y);
                case CardinalDirection.West:
                    return new Vector3<NEDGlobal>(v.x, -v.z, -v.y);
                default:
                    throw new NotSupportedException();
            }
        }

        public Quaternion<NEDGlobal> ToNED(Quaternion q)
        {
            switch (UnityZAxisDirection)
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
            return new Quaternion<NEDGlobal>(r.x, r.y, r.z, r.w);
        }

        public Vector3 FromNED(Vector3<NEDGlobal> v)
        {
            switch (UnityZAxisDirection)
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

        public Quaternion FromNED(Quaternion<NEDGlobal> q)
        {
            var r = new Quaternion(q.y, -q.z, q.x, -q.w);
            switch (UnityZAxisDirection)
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
