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

    public class Compass : MonoBehaviour
    {
        [SerializeField]
        CardinalDirection m_UnityZAxisDirection;
        public CardinalDirection UnityZAxisDirection
        {
            get => m_UnityZAxisDirection;
            set => m_UnityZAxisDirection = value;
        }

        public Vector3<ENU> ToENU(Vector3 v)
        {
            switch (m_UnityZAxisDirection)
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

        public Quaternion<ENU> ToENU(Quaternion q)
        {
            var r = Quaternion.Euler(0, 90 * ((int)m_UnityZAxisDirection - 1), 0) * q;
            return new Quaternion<ENU>(r.x, r.z, r.y, -r.w);
        }

        public Vector3 FromENU(Vector3<ENU> v)
        {
            switch (m_UnityZAxisDirection)
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

        public Quaternion FromENU(Quaternion<ENU> q)
        {
            var inverseRotationOffset = Quaternion.Euler(0, -90 * ((int)m_UnityZAxisDirection - 1), 0);
            return new Quaternion(q.x, q.z, q.y, -q.w) * inverseRotationOffset;
        }

        public Vector3<NED> ToNED(Vector3 v)
        {
            switch (m_UnityZAxisDirection)
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

        public Quaternion<NED> ToNED(Quaternion q)
        {
            var r = Quaternion.Euler(0, 90 * (int)m_UnityZAxisDirection, 0) * q;
            return new Quaternion<NED>(r.z, r.x, -r.y, -r.w);
        }

        public Vector3 FromNED(Vector3<NED> v)
        {
            switch (m_UnityZAxisDirection)
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

        public Quaternion FromNED(Quaternion<NED> q)
        {
            var inverseRotationOffset = Quaternion.Euler(0, -90 * (int)m_UnityZAxisDirection, 0);
            return new Quaternion(q.y, -q.z, q.x, -q.w) * inverseRotationOffset;
        }
    }
}
