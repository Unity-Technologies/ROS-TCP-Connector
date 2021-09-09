using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.ROSGeometry
{
    public interface ICoordinateSpace
    {
        Vector3 ConvertFromRUF(Vector3 v); // convert this vector from the Unity coordinate space into mine
        Vector3 ConvertToRUF(Vector3 v); // convert from my coordinate space into the Unity coordinate space

        Quaternion ConvertFromRUF(Quaternion q); // convert this quaternion from the Unity coordinate space into mine
        Quaternion ConvertToRUF(Quaternion q); // convert from my coordinate space into the Unity coordinate space
    }

    [Obsolete("CoordinateSpace has been renamed to ICoordinateSpace")]
    public interface CoordinateSpace : ICoordinateSpace
    {
    }

    /// <summary>
    /// RUF is the Unity coordinate space, so no conversion needed
    /// <list type="bullet">
    ///     <item><term>X axis: </term><description>Right and South</description></item>
    ///     <item><term>Y axis: </term><description>Up</description></item>
    ///     <item><term>Z axis: </term><description>Forward and East</description></item>
    /// </list>
    /// </summary>
    public class RUF : ICoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => v;
        public Vector3 ConvertToRUF(Vector3 v) => v;
        public Quaternion ConvertFromRUF(Quaternion q) => q;
        public Quaternion ConvertToRUF(Quaternion q) => q;
    }

    /// <summary>
    /// ROS standard forward, left, up (FLU) coordinate (REP-103)
    /// <list type="bullet">
    ///     <item><term>X axis: </term><description>Forward and East</description></item>
    ///     <item><term>Y axis: </term><description>Left and North</description></item>
    ///     <item><term>Z axis: </term><description>Up</description></item>
    /// </list>
    /// </summary>
    public class FLU : ICoordinateSpace
    {
        public Vector3 ConvertFromRUF(Vector3 v) => new Vector3(v.z, -v.x, v.y);
        public Vector3 ConvertToRUF(Vector3 v) => new Vector3(-v.y, v.z, v.x);
        public Quaternion ConvertFromRUF(Quaternion q) => new Quaternion(q.z, -q.x, q.y, -q.w);
        public Quaternion ConvertToRUF(Quaternion q) => new Quaternion(-q.y, q.z, q.x, -q.w);
    }

    /// <summary>
    /// Local north, east, down (NED) coordinates for outdoor systems, such as airplane and submarine (REP-103)
    /// <list type="bullet">
    ///     <item><term>X axis: </term><description>North</description></item>
    ///     <item><term>Y axis: </term><description>East</description></item>
    ///     <item><term>Z axis: </term><description>Down</description></item>
    /// </list>
    /// </summary>
    public class NED : ICoordinateSpace
    {
        private static Quaternion s_GeographicRotationOffset = Quaternion.Euler(0, 0, 90);
        private static Quaternion s_GeographicInverseRotationOffset = Quaternion.Euler(0, 0, -90);

        public Vector3 ConvertFromRUF(Vector3 v) => new Vector3(-v.x, v.z, -v.y);
        public Vector3 ConvertToRUF(Vector3 v) => new Vector3(-v.x, -v.z, v.y);
        public Quaternion ConvertFromRUF(Quaternion q)
        {
            // Identity quaternion is forwarding to East (z-axis) in RUF and it should face to North (x-axis) in NED
            var geoQuaternion = q * s_GeographicRotationOffset;
            return new Quaternion(-geoQuaternion.x, geoQuaternion.z, -geoQuaternion.y, -geoQuaternion.w);
        }
        public Quaternion ConvertToRUF(Quaternion q)
        {
            var r = new Quaternion(-q.x, -q.z, q.y, -q.w);
            // Identity quaternion if facing to North in NED and it should be rotated to East in RUF
            return r * s_GeographicInverseRotationOffset;
        }
    }

    /// <summary>
    /// Local east, north, up (ENU) coordinates for short-range Cartesian representations of geographic locations (REP-103)
    /// <list type="bullet">
    ///     <item><term>X axis: </term><description>East</description></item>
    ///     <item><term>Y axis: </term><description>North</description></item>
    ///     <item><term>Z axis: </term><description>Up</description></item>
    /// </list>
    /// </summary>
    public class ENU : FLU { }

    public enum CoordinateSpaceSelection
    {
        RUF,
        FLU,
        NED,
        ENU
    }

    public static class CoordinateSpaceExtensions
    {
        public static Vector3<C> To<C>(this Vector3 self)
            where C : ICoordinateSpace, new()
        {
            return new Vector3<C>(self);
        }

        public static Quaternion<C> To<C>(this Quaternion self)
            where C : ICoordinateSpace, new()
        {
            return new Quaternion<C>(self);
        }

        public static Vector3<C> As<C>(this PointMsg self) where C : ICoordinateSpace, new()
        {
            return new Vector3<C>((float)self.x, (float)self.y, (float)self.z);
        }

        public static Vector3 From<C>(this PointMsg self) where C : ICoordinateSpace, new()
        {
            return new Vector3<C>((float)self.x, (float)self.y, (float)self.z).toUnity;
        }

        public static Vector3<C> As<C>(this Point32Msg self) where C : ICoordinateSpace, new()
        {
            return new Vector3<C>(self.x, self.y, self.z);
        }

        public static Vector3 From<C>(this Point32Msg self) where C : ICoordinateSpace, new()
        {
            return new Vector3<C>(self.x, self.y, self.z).toUnity;
        }

        public static Vector3<C> As<C>(this Vector3Msg self) where C : ICoordinateSpace, new()
        {
            return new Vector3<C>((float)self.x, (float)self.y, (float)self.z);
        }

        public static Vector3 From<C>(this Vector3Msg self) where C : ICoordinateSpace, new()
        {
            return new Vector3<C>((float)self.x, (float)self.y, (float)self.z).toUnity;
        }

        public static Quaternion<C> As<C>(this QuaternionMsg self) where C : ICoordinateSpace, new()
        {
            return new Quaternion<C>((float)self.x, (float)self.y, (float)self.z, (float)self.w);
        }

        public static Quaternion From<C>(this QuaternionMsg self) where C : ICoordinateSpace, new()
        {
            return new Quaternion<C>((float)self.x, (float)self.y, (float)self.z, (float)self.w).toUnity;
        }

        public static TransformMsg To<C>(this Transform transform) where C : ICoordinateSpace, new()
        {
            return new TransformMsg(new Vector3<C>(transform.position), new Quaternion<C>(transform.rotation));
        }

        public static TransformMsg ToLocal<C>(this Transform transform) where C : ICoordinateSpace, new()
        {
            return new TransformMsg(new Vector3<C>(transform.localPosition), new Quaternion<C>(transform.localRotation));
        }

        public static Vector3 From(this PointMsg self, CoordinateSpaceSelection selection)
        {
            switch (selection)
            {
                case CoordinateSpaceSelection.RUF:
                    return self.From<RUF>();
                case CoordinateSpaceSelection.FLU:
                    return self.From<FLU>();
                case CoordinateSpaceSelection.ENU:
                    return self.From<ENU>();
                case CoordinateSpaceSelection.NED:
                    return self.From<NED>();
                default:
                    Debug.LogError("Invalid coordinate space " + selection);
                    return self.From<RUF>();
            }
        }

        public static Vector3 From(this Point32Msg self, CoordinateSpaceSelection selection)
        {
            switch (selection)
            {
                case CoordinateSpaceSelection.RUF:
                    return self.From<RUF>();
                case CoordinateSpaceSelection.FLU:
                    return self.From<FLU>();
                case CoordinateSpaceSelection.ENU:
                    return self.From<ENU>();
                case CoordinateSpaceSelection.NED:
                    return self.From<NED>();
                default:
                    Debug.LogError("Invalid coordinate space " + selection);
                    return self.From<RUF>();
            }
        }

        public static Vector3 From(this Vector3Msg self, CoordinateSpaceSelection selection)
        {
            switch (selection)
            {
                case CoordinateSpaceSelection.RUF:
                    return self.From<RUF>();
                case CoordinateSpaceSelection.FLU:
                    return self.From<FLU>();
                case CoordinateSpaceSelection.ENU:
                    return self.From<ENU>();
                case CoordinateSpaceSelection.NED:
                    return self.From<NED>();
                default:
                    Debug.LogError("Invalid coordinate space " + selection);
                    return self.From<RUF>();
            }
        }

        public static Quaternion From(this QuaternionMsg self, CoordinateSpaceSelection selection)
        {
            switch (selection)
            {
                case CoordinateSpaceSelection.RUF:
                    return self.From<RUF>();
                case CoordinateSpaceSelection.FLU:
                    return self.From<FLU>();
                case CoordinateSpaceSelection.ENU:
                    return self.From<ENU>();
                case CoordinateSpaceSelection.NED:
                    return self.From<NED>();
                default:
                    Debug.LogError("Invalid coordinate space " + selection);
                    return self.From<RUF>();
            }
        }
    }
}
