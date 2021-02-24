using RosMessageTypes.Geometry;
using System;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.ROSGeometry
{
    // This struct implements a feature-compatible version of the Unity Quaternion, but in a generic coordinate space.
    [Serializable]
    public struct Quaternion<C> where C : ICoordinateSpace, new()
    {
        [SerializeField]
        private Quaternion internalQuat;

        public float x { get => internalQuat.x; set => internalQuat.x = value; }
        public float y { get => internalQuat.y; set => internalQuat.y = value; }
        public float z { get => internalQuat.z; set => internalQuat.z = value; }
        public float w { get => internalQuat.w; set => internalQuat.w = value; }

        static C coordinateSpace = new C();

        public Quaternion(float x, float y, float z, float w)
        {
            internalQuat = new Quaternion(x, y, z, w);
        }

        // The actual coordinate-space conversion functions
        public Quaternion(Quaternion ruf)
        {
            internalQuat = coordinateSpace.ConvertFromRUF(ruf);
        }

        public Quaternion toUnity => coordinateSpace.ConvertToRUF(internalQuat);

        public static implicit operator MQuaternion(Quaternion<C> quat) => new MQuaternion(quat.x, quat.y, quat.z, quat.w);
        public static explicit operator Quaternion<C>(Quaternion quat) => new Quaternion<C>(quat);
        public static explicit operator Quaternion(Quaternion<C> rquat) => rquat.toUnity;

        public Quaternion<C2> To<C2>() where C2 : ICoordinateSpace, new()
        {
            return new Quaternion<C2>(this.toUnity);
        }

        // for internal use only - this function does not convert from Unity to ROS coordinate space
        private static Quaternion<C> MakeInternal(Quaternion q)
        {
            Quaternion<C> result = new Quaternion<C>();
            result.internalQuat = q;
            return result;
        }

        public static Vector3<C> operator *(Quaternion<C> rotation, Vector3<C> point)
        {
            Vector3 internalResult = rotation.internalQuat * new Vector3(point.x, point.y, point.z);
            return new Vector3<C>(internalResult.x, internalResult.y, internalResult.z);
        }

        public static Quaternion<C> operator *(Quaternion<C> lhs, Quaternion<C> rhs)
        {
            return MakeInternal(lhs.internalQuat * rhs.internalQuat);
        }

        public static float Angle(Quaternion<C> a, Quaternion<C> b) => Quaternion.Angle(a.internalQuat, b.internalQuat);

        public static Quaternion<C> AngleAxis(float angle, Vector3<C> axis)
        {
            return MakeInternal(Quaternion.AngleAxis(angle, new Vector3(axis.x, axis.y, axis.z)));
        }

        public static float Dot(Quaternion<C> a, Quaternion<C> b) => Quaternion.Dot(a.internalQuat, b.internalQuat);

        public static Quaternion<C> FromToRotation(Vector3<C> fromDirection, Vector3<C> toDirection)
        {
            return Quaternion.FromToRotation(fromDirection.toUnity, toDirection.toUnity).To<C>();
        }

        public static Quaternion<C> Inverse(Quaternion<C> rotation) => MakeInternal(Quaternion.Inverse(rotation.internalQuat));

        public static Quaternion<C> Lerp(Quaternion<C> a, Quaternion<C> b, float t)
        {
            return MakeInternal(Quaternion.Lerp(a.internalQuat, b.internalQuat, t));
        }

        public static Quaternion<C> LerpUnclamped(Quaternion<C> a, Quaternion<C> b, float t)
        {
            return MakeInternal(Quaternion.LerpUnclamped(a.internalQuat, b.internalQuat, t));
        }

        public static Quaternion<C> LookRotation(Vector3<C> forward)
        {
            return Quaternion.LookRotation(forward.toUnity).To<C>();
        }

        public static Quaternion<C> Normalize(Quaternion<C> q) => MakeInternal(Quaternion.Normalize(q.internalQuat));

        public static Quaternion<C> RotateTowards(Quaternion<C> from, Quaternion<C> to, float maxDegreesDelta)
        {
            return MakeInternal(Quaternion.RotateTowards(from.internalQuat, to.internalQuat, maxDegreesDelta));
        }

        public static Quaternion<C> Slerp(Quaternion<C> a, Quaternion<C> b, float t)
        {
            return MakeInternal(Quaternion.RotateTowards(a.internalQuat, b.internalQuat, t));
        }

        public static Quaternion<C> SlerpUnclamped(Quaternion<C> a, Quaternion<C> b, float t)
        {
            return MakeInternal(Quaternion.RotateTowards(a.internalQuat, b.internalQuat, t));
        }

        public bool Equals(Quaternion<C> other) => internalQuat == other.internalQuat;

        public override bool Equals(object other)
        {
            if (other is Quaternion<C>)
                return internalQuat == ((Quaternion<C>)other).internalQuat;
            return false;
        }

        public override int GetHashCode() => internalQuat.GetHashCode();
        public void Normalize() => internalQuat.Normalize();

        public void Set(float newX, float newY, float newZ, float newW)
        {
            internalQuat.Set(newX, newY, newZ, newW);
        }

        public void SetFromToRotation(Vector3<C> fromDirection, Vector3<C> toDirection)
        {
            this = Quaternion.FromToRotation(fromDirection.toUnity, toDirection.toUnity).To<C>();
        }

        public void SetLookRotation(Vector3<C> view)
        {
            this = Quaternion.LookRotation(view.toUnity).To<C>();
        }

        public void ToAngleAxis(out float angle, out Vector3<C> axis)
        {
            Vector3 uaxis;
            internalQuat.ToAngleAxis(out angle, out uaxis);
            axis = uaxis.To<C>();
        }

        public string ToString(string format, IFormatProvider formatProvider) => internalQuat.ToString(format, formatProvider);
        public string ToString(string format) => internalQuat.ToString(format);
        public override string ToString() => internalQuat.ToString();
    }
}
