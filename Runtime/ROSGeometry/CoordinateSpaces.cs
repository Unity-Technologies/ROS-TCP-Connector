using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ROSGeometry
{
    public interface CoordinateSpace
    {
        Vector3 ConvertFromRUF(Vector3 v); // convert this vector from the Unity coordinate space into mine
        Vector3 ConvertToRUF(Vector3 v); // convert from my coordinate space into the Unity coordinate space

        Quaternion ConvertFromRUF(Quaternion q); // convert this quaternion from the Unity coordinate space into mine
        Quaternion ConvertToRUF(Quaternion q); // convert from my coordinate space into the Unity coordinate space
    }

    //RUF is the Unity coordinate space, so no conversion needed
    public class RUF : CoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => v;
        public Vector3 ConvertToRUF(Vector3 v) => v;
        public Quaternion ConvertFromRUF(Quaternion q) => q;
        public Quaternion ConvertToRUF(Quaternion q) => q;
    }

    public class FLU : CoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => new Vector3(v.z, -v.x, v.y);
        public Vector3 ConvertToRUF(Vector3 v) => new Vector3(-v.y, v.z, v.x);
        public Quaternion ConvertFromRUF(Quaternion q) => new Quaternion(q.z, -q.x, q.y, -q.w);
        public Quaternion ConvertToRUF(Quaternion q) => new Quaternion(-q.y, q.z, q.x, -q.w);
    }

    public class NED : CoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => new Vector3(v.z, v.x, -v.y);
        public Vector3 ConvertToRUF(Vector3 v) => new Vector3(v.y, -v.z, v.x);
        public Quaternion ConvertFromRUF(Quaternion q) => new Quaternion(q.z, q.x, -q.y, -q.w);
        public Quaternion ConvertToRUF(Quaternion q) => new Quaternion(q.y, -q.z, q.x, -q.w);
    }

    public class ENU : CoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => new Vector3(v.x, v.z, v.y);
        public Vector3 ConvertToRUF(Vector3 v) => new Vector3(v.x, v.z, v.y);
        public Quaternion ConvertFromRUF(Quaternion q) => new Quaternion(q.x, q.z, q.y, -q.w);
        public Quaternion ConvertToRUF(Quaternion q) => new Quaternion(q.x, q.z, q.y, -q.w);
    }

    public static class CoordinateSpaceExtensions
    {
        public static Vector3<C> To<C>(this Vector3 self)
            where C : CoordinateSpace, new()
        {
            return new Vector3<C>(self);
        }

        public static Quaternion<C> To<C>(this Quaternion self)
            where C : CoordinateSpace, new()
        {
            return new Quaternion<C>(self);
        }

        public static RosMessageTypes.Geometry.Transform To<C>(this Transform transform) where C : CoordinateSpace, new()
        {
            return new RosMessageTypes.Geometry.Transform(new Vector3<C>(transform.position), new Quaternion<C>(transform.rotation));
        }
    }
}