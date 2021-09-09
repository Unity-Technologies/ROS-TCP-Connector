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

    public enum GeographicCoordinate
    {
        ENU,
        NED,
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

        [SerializeField]
        GeographicCoordinate m_Coordinate;
        public GeographicCoordinate Coordinate
        {
            get => m_Coordinate;
            set => m_Coordinate = value;
        }

        BaseCompass compass;

        public Vector3 FromUnity(Vector3 v) => compass.FromUnity(v);
        public Quaternion FromUnity(Quaternion q) => compass.FromUnity(q);
        public Vector3 ToUnity(Vector3 v) => compass.ToUnity(v);
        public Quaternion ToUnity(Quaternion q) => compass.ToUnity(q);

        void OnValidate()
        {
            Initialize();
        }

        void Start()
        {
            if (compass == null)
            {
                Initialize();
            }
        }

        void Initialize()
        {
            switch (Coordinate)
            {
                case GeographicCoordinate.ENU:
                    compass = new EnuCompass(m_UnityZAxisDirection);
                    break;
                case GeographicCoordinate.NED:
                    compass = new NedCompass(m_UnityZAxisDirection);
                    break;
            }
        }
    }

    abstract class BaseCompass
    {
        public CardinalDirection m_UnityZAxisDirection;

        public abstract Vector3 FromUnity(Vector3 v);       // convert this vector from the Unity coordinate space into geo coordinate
        public abstract Quaternion FromUnity(Quaternion q); // convert this quaternion from the Unity coordinate space into geo coordinate
        public abstract Vector3 ToUnity(Vector3 v);         // convert this vector from the geo coordinate space into Unity coordinate
        public abstract Quaternion ToUnity(Quaternion q);   // convert this quaternion from the geo coordinate space into Unity coordinate
    }

    class EnuCompass : BaseCompass
    {
        public EnuCompass(CardinalDirection zDirection)
        {
            m_UnityZAxisDirection = zDirection;
        }

        public override Vector3 FromUnity(Vector3 v)
        {
            switch (m_UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Vector3(v.x, v.z, v.y);
                case CardinalDirection.East:
                    return new Vector3(v.z, -v.x, v.y);
                case CardinalDirection.South:
                    return new Vector3(-v.x, -v.z, v.y);
                case CardinalDirection.West:
                    return new Vector3(-v.z, v.x, v.y);
                default:
                    throw new NotSupportedException();
            }
        }

        public override Quaternion FromUnity(Quaternion q)
        {
            var r = q * Quaternion.Euler(0, 90 * ((int)m_UnityZAxisDirection - 1), 0);
            switch (m_UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Quaternion(r.x, r.z, r.y, -r.w);
                case CardinalDirection.East:
                    return new Quaternion(r.z, -r.x, r.y, -r.w);
                case CardinalDirection.South:
                    return new Quaternion(-r.x, -r.z, r.y, -r.w);
                case CardinalDirection.West:
                    return new Quaternion(-r.z, r.x, r.y, -r.w);
                default:
                    throw new NotSupportedException();
            }
        }

        public override Vector3 ToUnity(Vector3 v)
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

        public override Quaternion ToUnity(Quaternion q)
        {
            var inverseRotationOffset = Quaternion.Euler(0, -90 * ((int)m_UnityZAxisDirection - 1), 0);
            switch (m_UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Quaternion(q.x, q.z, q.y, -q.w) * inverseRotationOffset;
                case CardinalDirection.East:
                    return new Quaternion(-q.y, q.z, q.x, -q.w) * inverseRotationOffset;
                case CardinalDirection.South:
                    return new Quaternion(-q.x, q.z, -q.y, -q.w) * inverseRotationOffset;
                case CardinalDirection.West:
                    return new Quaternion(q.y, q.z, -q.x, -q.w) * inverseRotationOffset;
                default:
                    throw new NotSupportedException();
            }
        }
    }

    class NedCompass : BaseCompass
    {
        public NedCompass(CardinalDirection zDirection)
        {
            m_UnityZAxisDirection = zDirection;
        }

        public override Vector3 FromUnity(Vector3 v)
        {
            switch (m_UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Vector3(v.z, v.x, -v.y);
                case CardinalDirection.East:
                    return new Vector3(-v.x, v.z, -v.y);
                case CardinalDirection.South:
                    return new Vector3(-v.z, -v.x, -v.y);
                case CardinalDirection.West:
                    return new Vector3(v.x, -v.z, -v.y);
                default:
                    throw new NotSupportedException();
            }
        }

        public override Quaternion FromUnity(Quaternion q)
        {
            var r = q * Quaternion.Euler(0, 90 * (int)m_UnityZAxisDirection, 0);
            switch (m_UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Quaternion(r.z, r.x, -r.y, -r.w);
                case CardinalDirection.East:
                    return new Quaternion(-r.x, r.z, -r.y, -r.w);
                case CardinalDirection.South:
                    return new Quaternion(-r.z, -r.x, -r.y, -r.w);
                case CardinalDirection.West:
                    return new Quaternion(r.x, -r.z, -r.y, -r.w);
                default:
                    throw new NotSupportedException();
            }
        }

        public override Vector3 ToUnity(Vector3 v)
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

        public override Quaternion ToUnity(Quaternion q)
        {
            var inverseRotationOffset = Quaternion.Euler(0, -90 * (int)m_UnityZAxisDirection, 0);
            switch (m_UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return new Quaternion(q.y, -q.z, q.x, -q.w) * inverseRotationOffset;
                case CardinalDirection.East:
                    return new Quaternion(-q.x, -q.z, q.y, -q.w) * inverseRotationOffset;
                case CardinalDirection.South:
                    return new Quaternion(-q.y, -q.z, -q.x, -q.w) * inverseRotationOffset;
                case CardinalDirection.West:
                    return new Quaternion(q.x, -q.z, -q.y, -q.w) * inverseRotationOffset;
                default:
                    throw new NotSupportedException();
            }
        }
    }
}
