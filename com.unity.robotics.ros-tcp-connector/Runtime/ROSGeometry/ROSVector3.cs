using System;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.ROSGeometry
{
    // This struct implements a feature-compatible version of the Unity Vector3, but in a generic coordinate space.
    [Serializable]
    public struct Vector3<C> : IEquatable<Vector3<C>>, IFormattable where C : ICoordinateSpace, new()
    {
        // For implementation convenience this actually contains a real Unity Vector3. Its components are in ROS coordinate space.
        // For safety, this is private and users should never need to access or know about it.
        // Other Vector3s should always be in Unity coordinate space.
        [SerializeField]
        private Vector3 internalVector;
        public float x { get => internalVector.x; set => internalVector.x = value; }
        public float y { get => internalVector.y; set => internalVector.y = value; }
        public float z { get => internalVector.z; set => internalVector.z = value; }

        static C coordinateSpace = new C();

        public Vector3(float x, float y, float z)
        {
            internalVector = new Vector3(x, y, z);
        }

        public Vector3(Vector3 vec_ruf)
        {
            internalVector = coordinateSpace.ConvertFromRUF(vec_ruf);
        }

        public Vector3 toUnity => coordinateSpace.ConvertToRUF(internalVector);

        public static explicit operator Vector3<C>(Vector3 vec)
        {
            return MakeInternal(coordinateSpace.ConvertFromRUF(vec));
        }

        public static explicit operator Vector3(Vector3<C> vec)
        {
            return coordinateSpace.ConvertToRUF(vec.internalVector);
        }

        public Vector3<C2> To<C2>() where C2 : ICoordinateSpace, new()
        {
            return (Vector3<C2>)(Vector3)this;
        }

        // for internal use only - this function does not convert coordinate spaces correctly
        private static Vector3<C> MakeInternal(Vector3 vec)
        {
            Vector3<C> result = new Vector3<C>();
            result.internalVector = vec;
            return result;
        }

        public static implicit operator RosMessageTypes.Geometry.Point(Vector3<C> rvec) => new RosMessageTypes.Geometry.Point(rvec.x, rvec.y, rvec.z);
        public static implicit operator RosMessageTypes.Geometry.Point32(Vector3<C> rvec) => new RosMessageTypes.Geometry.Point32(rvec.x, rvec.y, rvec.z);
        public static implicit operator RosMessageTypes.Geometry.Vector3(Vector3<C> rvec) => new RosMessageTypes.Geometry.Vector3(rvec.x, rvec.y, rvec.z);

        public static Vector3<C> right => new Vector3<C>(Vector3.right);
        public static Vector3<C> left => new Vector3<C>(Vector3.left);
        public static Vector3<C> up => new Vector3<C>(Vector3.up);
        public static Vector3<C> back => new Vector3<C>(Vector3.back);
        public static Vector3<C> forward => new Vector3<C>(Vector3.forward);
        public static Vector3<C> down => new Vector3<C>(Vector3.down);
        public static Vector3<C> one => MakeInternal(Vector3.one);
        public static Vector3<C> zero => MakeInternal(Vector3.zero);
        public static Vector3<C> negativeInfinity => MakeInternal(Vector3.negativeInfinity);
        public static Vector3<C> positiveInfinity => MakeInternal(Vector3.positiveInfinity);
        public Vector3<C> normalized => MakeInternal(internalVector.normalized);
        public float magnitude => internalVector.magnitude;
        public float sqrMagnitude => internalVector.sqrMagnitude;

        public static float Angle(Vector3<C> from, Vector3<C> to) => Vector3.Angle(from.internalVector, to.internalVector);
        
        public static Vector3<C> ClampMagnitude(Vector3<C> vector, float maxLength)
        {
            return MakeInternal(Vector3.ClampMagnitude(vector.internalVector, maxLength));
        }

        public static Vector3<C> Cross(Vector3<C> lhs, Vector3<C> rhs)
        {
            return MakeInternal(Vector3.Cross(lhs.internalVector, rhs.internalVector));
        }

        public static float Distance(Vector3<C> a, Vector3<C> b) => Vector3.Distance(a.internalVector, b.internalVector);
        
        public static float Dot(Vector3<C> lhs, Vector3<C> rhs) => Vector3.Dot(lhs.internalVector, rhs.internalVector);
        
        public static Vector3<C> Lerp(Vector3<C> a, Vector3<C> b, float t)
        {
            return MakeInternal(Vector3.Lerp(a.internalVector, b.internalVector, t));
        }
        
        public static Vector3<C> LerpUnclamped(Vector3<C> a, Vector3<C> b, float t)
        {
            return MakeInternal(Vector3.LerpUnclamped(a.internalVector, b.internalVector, t));
        }
        
        public static float Magnitude(Vector3<C> vector) => Vector3.Magnitude(vector.internalVector);
        
        public static Vector3<C> Max(Vector3<C> lhs, Vector3<C> rhs) => MakeInternal(Vector3.Max(lhs.internalVector, rhs.internalVector));
        
        public static Vector3<C> Min(Vector3<C> lhs, Vector3<C> rhs) => MakeInternal(Vector3.Min(lhs.internalVector, rhs.internalVector));
        
        public static Vector3<C> MoveTowards(Vector3<C> current, Vector3<C> target, float maxDistanceDelta)
        {
            return MakeInternal(Vector3.MoveTowards(current.internalVector, target.internalVector, maxDistanceDelta));
        }
        
        public static Vector3<C> Normalize(Vector3<C> value) => MakeInternal(Vector3.Normalize(value.internalVector));
        
        public static void OrthoNormalize(ref Vector3<C> normal, ref Vector3<C> tangent, ref Vector3<C> binormal)
        {
            Vector3.OrthoNormalize(ref normal.internalVector, ref tangent.internalVector, ref binormal.internalVector);
        }
        
        public static void OrthoNormalize(ref Vector3<C> normal, ref Vector3<C> tangent)
        {
            Vector3.OrthoNormalize(ref normal.internalVector, ref tangent.internalVector);
        }
        
        public static Vector3<C> Project(Vector3<C> vector, Vector3<C> onNormal)
        {
            return MakeInternal(Vector3.Project(vector.internalVector, onNormal.internalVector));
        }
        
        public static Vector3<C> ProjectOnPlane(Vector3<C> vector, Vector3<C> planeNormal)
        {
            return MakeInternal(Vector3.ProjectOnPlane(vector.internalVector, planeNormal.internalVector));
        }
        
        public static Vector3<C> Reflect(Vector3<C> inDirection, Vector3<C> inNormal)
        {
            return MakeInternal(Vector3.Reflect(inDirection.internalVector, inNormal.internalVector));
        }
        
        public static Vector3<C> RotateTowards(Vector3<C> current, Vector3<C> target, float maxRadiansDelta, float maxMagnitudeDelta)
        {
            return MakeInternal(Vector3.RotateTowards(current.internalVector, target.internalVector, maxRadiansDelta, maxMagnitudeDelta));
        }
        
        public static Vector3<C> Scale(Vector3<C> a, Vector3<C> b) => MakeInternal(Vector3.Scale(a.internalVector, b.internalVector));
        
        public static float SignedAngle(Vector3<C> from, Vector3<C> to, Vector3<C> axis)
        {
            return Vector3.SignedAngle(from.internalVector, to.internalVector, axis.internalVector);
        }
        
        public static Vector3<C> Slerp(Vector3<C> a, Vector3<C> b, float t)
        {
            return MakeInternal(Vector3.Slerp(a.internalVector, b.internalVector, t));
        }
        
        public static Vector3<C> SlerpUnclamped(Vector3<C> a, Vector3<C> b, float t)
        {
            return MakeInternal(Vector3.SlerpUnclamped(a.internalVector, b.internalVector, t));
        }
        
        public static Vector3<C> SmoothDamp(Vector3<C> current, Vector3<C> target, ref Vector3<C> currentVelocity, float smoothTime)
        {
            return MakeInternal(Vector3.SmoothDamp(current.internalVector, target.internalVector, ref currentVelocity.internalVector, smoothTime));
        }
        
        public static Vector3<C> SmoothDamp(Vector3<C> current, Vector3<C> target, ref Vector3<C> currentVelocity, float smoothTime, float maxSpeed)
        {
            return MakeInternal(Vector3.SmoothDamp(current.internalVector, target.internalVector, ref currentVelocity.internalVector, smoothTime, maxSpeed));
        }
        
        public static float SqrMagnitude(Vector3<C> vector) => Vector3.SqrMagnitude(vector.internalVector);
        
        public override bool Equals(object other)
        {
            if (other is Vector3<C>)
                return internalVector == ((Vector3<C>)other).internalVector;
            return false;
        }
        
        public bool Equals(Vector3<C> other) => internalVector == other.internalVector;
        
        public override int GetHashCode() => internalVector.GetHashCode();
        
        public void Normalize()
        {
            internalVector.Normalize();
        }

        public void Scale(Vector3<C> scale)
        {
            internalVector.Scale(scale.internalVector);
        }
        
        public void Set(float newX, float newY, float newZ)
        {
            internalVector.Set(newX, newY, newZ);
        }
        
        public string ToString(string format) => internalVector.ToString(format);
        public override string ToString() => internalVector.ToString();
        public string ToString(string format, System.IFormatProvider formatProvider) => internalVector.ToString(format, formatProvider);

        public static Vector3<C> operator +(Vector3<C> a, Vector3<C> b)
        {
            return MakeInternal(a.internalVector + b.internalVector);
        }
        
        public static Vector3<C> operator -(Vector3<C> a)
        {
            return MakeInternal(-a.internalVector);
        }

        public static Vector3<C> operator -(Vector3<C> a, Vector3<C> b)
        {
            return MakeInternal(a.internalVector - b.internalVector);
        }
        
        public static Vector3<C> operator *(float d, Vector3<C> a)
        {
            return MakeInternal(d * a.internalVector);
        }
        
        public static Vector3<C> operator *(Vector3<C> a, float d)
        {
            return MakeInternal(a.internalVector * d);
        }
        
        public static Vector3<C> operator /(Vector3<C> a, float d)
        {
            return MakeInternal(a.internalVector / d);
        }
        
        public static bool operator ==(Vector3<C> lhs, Vector3<C> rhs)
        {
            return lhs.internalVector == rhs.internalVector;
        }

        public static bool operator !=(Vector3<C> lhs, Vector3<C> rhs)
        {
            return lhs.internalVector != rhs.internalVector;
        }
    }
}
