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

    public static class GeometryCompass
    {
        public static CardinalDirection UnityZAxisDirection = CardinalDirection.North;

        private static Quaternion s_NinetyYaw = Quaternion.Euler(0, 90, 0);
        private static Quaternion s_OneEightyYaw = Quaternion.Euler(0, 180, 0);
        private static Quaternion s_NegativeNinetyYaw = Quaternion.Euler(0, -90, 0);

        public static Vector3<ENU> ToENU(Vector3 v)
        {
            switch (UnityZAxisDirection)
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
            var r = Quaternion.Euler(0, 90 * ((int)UnityZAxisDirection - 1), 0) * q;
            var p = r.To<FLU>();
            return new Quaternion<ENU>(p.x, p.y, p.z, -p.w);
            return new Quaternion<ENU>(r.x, r.z, r.y, -r.w);
        }

        public static Vector3 FromENU(Vector3<ENU> v)
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

        public static Quaternion FromENU(Quaternion<ENU> q)
        {
            var inverseRotationOffset = Quaternion.Euler(0, -90 * ((int)UnityZAxisDirection - 1), 0);
            return inverseRotationOffset * new Quaternion(q.x, q.z, q.y, -q.w);
        }

        public static Vector3<NED> ToNED(Vector3 v)
        {
            switch (UnityZAxisDirection)
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
            var r = Quaternion.Euler(0, 90 * (int)UnityZAxisDirection, 0) * q;
            return new Quaternion<NED>(r.z, r.x, -r.y, -r.w);
        }

        public static Vector3 FromNED(Vector3<NED> v)
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

        public static Quaternion FromNED(Quaternion<NED> q)
        {
            var inverseRotationOffset = Quaternion.Euler(0, -90 * (int)UnityZAxisDirection, 0);
            return inverseRotationOffset * new Quaternion(q.y, -q.z, q.x, -q.w);
        }
    }
}
