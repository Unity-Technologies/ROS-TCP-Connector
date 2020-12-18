using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ROSGeometry
{
    /// <summary>
    /// Represents a coordinate space for a ROSGeometry.Vector3 or Quaternion
    /// </summary>
    public interface CoordinateSpace
    {
        /// <summary>
        /// Convert this Vector3 from the Unity coordinate space into mine
        /// </summary>
        /// <param name="v">The vector to convert</param>
        /// <returns>The result</returns>
        Vector3 ConvertFromRUF(Vector3 v);
        /// <summary>
        /// Convert this Vector3 from my coordinate space into Unity's
        /// </summary>
        /// <param name="v">The vector to convert</param>
        /// <returns>The result</returns>
        Vector3 ConvertToRUF(Vector3 v);

        /// <summary>
        /// Convert this Quaternion from the Unity coordinate space into mine
        /// </summary>
        /// <param name="q">The quaternion to convert</param>
        /// <returns>The result</returns>
        Quaternion ConvertFromRUF(Quaternion q);

        /// <summary>
        /// Convert this Quaternion from my coordinate space into Unity's
        /// </summary>
        /// <param name="q">The quaternion to convert</param>
        /// <returns>The result</returns>
        Quaternion ConvertToRUF(Quaternion q);
    }

    /// <summary>
    /// The native Unity coordinate frame: X Right, Y Up, Z Forward
    /// </summary>
    public class RUF : CoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => v;
        public Vector3 ConvertToRUF(Vector3 v) => v;
        public Quaternion ConvertFromRUF(Quaternion q) => q;
        public Quaternion ConvertToRUF(Quaternion q) => q;
    }

    /// <summary>
    /// The default ROS coordinate frame: X Forward, Y Left, Z Up
    /// </summary>
    public class FLU : CoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => new Vector3(v.z, -v.x, v.y);
        public Vector3 ConvertToRUF(Vector3 v) => new Vector3(-v.y, v.z, v.x);
        public Quaternion ConvertFromRUF(Quaternion q) => new Quaternion(q.z, -q.x, q.y, -q.w);
        public Quaternion ConvertToRUF(Quaternion q) => new Quaternion(-q.y, q.z, q.x, -q.w);
    }

    /// <summary>
    /// Coordinate frame for aviation coordinates: X North (forward), Y East (right), Z Down
    /// </summary>
    public class NED : CoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => new Vector3(v.z, v.x, -v.y);
        public Vector3 ConvertToRUF(Vector3 v) => new Vector3(v.y, -v.z, v.x);
        public Quaternion ConvertFromRUF(Quaternion q) => new Quaternion(q.z, q.x, -q.y, -q.w);
        public Quaternion ConvertToRUF(Quaternion q) => new Quaternion(q.y, -q.z, q.x, -q.w);
    }

    /// <summary>
    /// Coordinate frame for geographic locations: X East (right), Y North (forward), Z Up
    /// </summary>
    public class ENU : CoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => new Vector3(v.x, v.z, v.y);
        public Vector3 ConvertToRUF(Vector3 v) => new Vector3(v.x, v.z, v.y);
        public Quaternion ConvertFromRUF(Quaternion q) => new Quaternion(q.x, q.z, q.y, -q.w);
        public Quaternion ConvertToRUF(Quaternion q) => new Quaternion(q.x, q.z, q.y, -q.w);
    }

    /// <summary>
    /// Extension methods for working with ROSGeometry's Vector3 and Quaternion types
    /// </summary>
    public static class CoordinateSpaceExtensions
    {
        /// <summary>
        /// Convert this Unity Vector3 into a Vector3&ltC;&gt; in the target coordinate space
        /// </summary>
        /// <typeparam name="C">The target coordinate space</typeparam>
        /// <param name="self">The Unity Vector3 to convert</param>
        /// <returns>The converted Vector3&lt;C&gt;</returns>
        public static Vector3<C> To<C>(this Vector3 self)
            where C : CoordinateSpace, new()
        {
            return new Vector3<C>(self);
        }

        /// <summary>
        /// Convert this Unity Quaternion into a Quaternion&ltC;&gt; in the target coordinate space
        /// </summary>
        /// <typeparam name="C">The target coordinate space</typeparam>
        /// <param name="self">The Unity Quaternion to convert</param>
        /// <returns>The converted Quaternion&lt;C&gt;</returns>
        public static Quaternion<C> To<C>(this Quaternion self)
            where C : CoordinateSpace, new()
        {
            return new Quaternion<C>(self);
        }

        public static Vector3<C> As<C>(this RosMessageTypes.Geometry.Point self) where C : CoordinateSpace, new()
        {
            return new Vector3<C>((float)self.x, (float)self.y, (float)self.z);
        }

        public static Vector3 From<C>(this RosMessageTypes.Geometry.Point self) where C : CoordinateSpace, new()
        {
            return new Vector3<C>((float)self.x, (float)self.y, (float)self.z).toUnity;
        }

        public static Vector3<C> As<C>(this RosMessageTypes.Geometry.Point32 self) where C : CoordinateSpace, new()
        {
            return new Vector3<C>(self.x, self.y, self.z);
        }

        public static Vector3 From<C>(this RosMessageTypes.Geometry.Point32 self) where C : CoordinateSpace, new()
        {
            return new Vector3<C>(self.x, self.y, self.z).toUnity;
        }

        public static Vector3<C> As<C>(this RosMessageTypes.Geometry.Vector3 self) where C : CoordinateSpace, new()
        {
            return new Vector3<C>((float)self.x, (float)self.y, (float)self.z);
        }

        public static Vector3 From<C>(this RosMessageTypes.Geometry.Vector3 self) where C : CoordinateSpace, new()
        {
            return new Vector3<C>((float)self.x, (float)self.y, (float)self.z).toUnity;
        }

        public static Quaternion<C> As<C>(this RosMessageTypes.Geometry.Quaternion self) where C : CoordinateSpace, new()
        {
            return new Quaternion<C>((float)self.x, (float)self.y, (float)self.z, (float)self.w);
        }

        public static Quaternion From<C>(this RosMessageTypes.Geometry.Quaternion self) where C : CoordinateSpace, new()
        {
            return new Quaternion<C>((float)self.x, (float)self.y, (float)self.z, (float)self.w).toUnity;
        }

        public static RosMessageTypes.Geometry.Transform To<C>(this Transform transform) where C : CoordinateSpace, new()
        {
            return new RosMessageTypes.Geometry.Transform(new Vector3<C>(transform.position), new Quaternion<C>(transform.rotation));
        }
    }
}