using RosMessageTypes.Geometry;
using System;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.ROSGeometry
{
    public interface ICoordinateSpace
    {
        Vector3 ConvertFromRUF(Vector3 v); // convert this vector from the Unity coordinate space into mine
        Vector3 ConvertToRUF(Vector3 v); // convert from my coordinate space into the Unity coordinate space

        Quaternion ConvertFromRUF(Quaternion q); // convert this quaternion from the Unity coordinate space into mine
        Quaternion ConvertToRUF(Quaternion q); // convert from my coordinate space into the Unity coordinate space

        Vector3 ConvertAngularVelocityFromRUF(Vector3 angularVelocity); // convert this angular velocity from the Unity coordinate space into mine
        Vector3 ConvertAngularVelocityToRUF(Vector3 angularVelocity); // convert from my coordinate space into the Unity coordinate space
    }

    [Obsolete("CoordinateSpace has been renamed to ICoordinateSpace")]
    public interface CoordinateSpace : ICoordinateSpace
    {
    }

    //RUF is the Unity coordinate space, so no conversion needed
    public class RUF : ICoordinateSpace
    {
        Vector3 ICoordinateSpace.ConvertFromRUF(Vector3 v) => v;
        Vector3 ICoordinateSpace.ConvertToRUF(Vector3 v) => v;
        Quaternion ICoordinateSpace.ConvertFromRUF(Quaternion q) => q;
        Quaternion ICoordinateSpace.ConvertToRUF(Quaternion q) => q;
        Vector3 ICoordinateSpace.ConvertAngularVelocityFromRUF(Vector3 angularVelocity) => angularVelocity;
        Vector3 ICoordinateSpace.ConvertAngularVelocityToRUF(Vector3 angularVelocity) => angularVelocity;
    }

    public class FLU : ICoordinateSpace
    {
        public static Vector3 ConvertFromRUF(Vector3 v) => new Vector3(v.z, -v.x, v.y);
        public static Vector3 ConvertToRUF(Vector3 v) => new Vector3(-v.y, v.z, v.x);
        public static Quaternion ConvertFromRUF(Quaternion q) => new Quaternion(q.z, -q.x, q.y, -q.w);
        public static Quaternion ConvertToRUF(Quaternion q) => new Quaternion(-q.y, q.z, q.x, -q.w);
        public static Vector3 ConvertAngularVelocityFromRUF(Vector3 angularVelocity) => -FLU.ConvertFromRUF(angularVelocity);
        public static Vector3 ConvertAngularVelocityToRUF(Vector3 angularVelocity) => -FLU.ConvertToRUF(angularVelocity);

        Vector3 ICoordinateSpace.ConvertFromRUF(Vector3 v) => FLU.ConvertFromRUF(v);
        Vector3 ICoordinateSpace.ConvertToRUF(Vector3 v) => FLU.ConvertToRUF(v);
        Quaternion ICoordinateSpace.ConvertFromRUF(Quaternion q) => FLU.ConvertFromRUF(q);
        Quaternion ICoordinateSpace.ConvertToRUF(Quaternion q) => FLU.ConvertToRUF(q);
        Vector3 ICoordinateSpace.ConvertAngularVelocityFromRUF(Vector3 angularVelocity) => FLU.ConvertAngularVelocityFromRUF(angularVelocity);
        Vector3 ICoordinateSpace.ConvertAngularVelocityToRUF(Vector3 angularVelocity) => FLU.ConvertAngularVelocityToRUF(angularVelocity);
    }

    public class ENULocal : FLU { }

    public class FRD : ICoordinateSpace
    {
        public static Vector3 ConvertFromRUF(Vector3 v) => new Vector3(v.z, v.x, -v.y);
        public static Vector3 ConvertToRUF(Vector3 v) => new Vector3(v.y, -v.z, v.x);
        public static Quaternion ConvertFromRUF(Quaternion q) => new Quaternion(q.z, q.x, -q.y, -q.w);
        public static Quaternion ConvertToRUF(Quaternion q) => new Quaternion(q.y, -q.z, q.x, -q.w);
        public static Vector3 ConvertAngularVelocityFromRUF(Vector3 angularVelocity) => -ConvertFromRUF(angularVelocity);
        public static Vector3 ConvertAngularVelocityToRUF(Vector3 angularVelocity) => -ConvertToRUF(angularVelocity);

        Vector3 ICoordinateSpace.ConvertFromRUF(Vector3 v) => FRD.ConvertFromRUF(v);
        Vector3 ICoordinateSpace.ConvertToRUF(Vector3 v) => FRD.ConvertToRUF(v);
        Quaternion ICoordinateSpace.ConvertFromRUF(Quaternion q) => FRD.ConvertFromRUF(q);
        Quaternion ICoordinateSpace.ConvertToRUF(Quaternion q) => FRD.ConvertToRUF(q);
        Vector3 ICoordinateSpace.ConvertAngularVelocityFromRUF(Vector3 angularVelocity) => FRD.ConvertAngularVelocityFromRUF(angularVelocity);
        Vector3 ICoordinateSpace.ConvertAngularVelocityToRUF(Vector3 angularVelocity) => FRD.ConvertAngularVelocityToRUF(angularVelocity);
    }

    public class NEDLocal : FRD { }

    public class NED : ICoordinateSpace
    {
        public static Vector3 ConvertFromRUF(Vector3 v)
        {
            switch (GeometryCompass.UnityZAxisDirection)
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

        public static Vector3 ConvertToRUF(Vector3 v)
        {
            switch (GeometryCompass.UnityZAxisDirection)
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

        public static Quaternion ConvertFromRUF(Quaternion q)
        {
            switch (GeometryCompass.UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    break;
                case CardinalDirection.East:
                    q = GeometryCompass.k_NinetyYaw * q;
                    break;
                case CardinalDirection.South:
                    q = GeometryCompass.k_OneEightyYaw * q;
                    break;
                case CardinalDirection.West:
                    q = GeometryCompass.k_NegativeNinetyYaw * q;
                    break;
                default:
                    throw new NotSupportedException();
            }
            return new Quaternion(q.z, q.x, -q.y, -q.w);
        }

        public static Quaternion ConvertToRUF(Quaternion q)
        {
            var r = new Quaternion(q.y, -q.z, q.x, -q.w);
            switch (GeometryCompass.UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return r;
                case CardinalDirection.East:
                    return GeometryCompass.k_NegativeNinetyYaw * r;
                case CardinalDirection.South:
                    return GeometryCompass.k_OneEightyYaw * r;
                case CardinalDirection.West:
                    return GeometryCompass.k_NinetyYaw * r;
                default:
                    throw new NotSupportedException();
            }
        }
        Vector3 ICoordinateSpace.ConvertFromRUF(Vector3 v) => NED.ConvertFromRUF(v);
        Vector3 ICoordinateSpace.ConvertToRUF(Vector3 v) => NED.ConvertToRUF(v);
        Quaternion ICoordinateSpace.ConvertFromRUF(Quaternion q) => NED.ConvertFromRUF(q);
        Quaternion ICoordinateSpace.ConvertToRUF(Quaternion q) => NED.ConvertToRUF(q);
        //Note: Angular Velocity is the same as FRD / NEDLocal
        Vector3 ICoordinateSpace.ConvertAngularVelocityFromRUF(Vector3 angularVelocity) => FRD.ConvertAngularVelocityFromRUF(angularVelocity);
        Vector3 ICoordinateSpace.ConvertAngularVelocityToRUF(Vector3 angularVelocity) => FRD.ConvertAngularVelocityToRUF(angularVelocity);
    }

    public class ENU : ICoordinateSpace
    {
        public static Vector3 ConvertFromRUF(Vector3 v)
        {
            switch (GeometryCompass.UnityZAxisDirection)
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
        public static Vector3 ConvertToRUF(Vector3 v)
        {
            switch (GeometryCompass.UnityZAxisDirection)
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

        public static Quaternion ConvertFromRUF(Quaternion q)
        {
            switch (GeometryCompass.UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    q = GeometryCompass.k_NegativeNinetyYaw * q;
                    break;
                case CardinalDirection.East:
                    break;
                case CardinalDirection.South:
                    q = GeometryCompass.k_NinetyYaw * q;
                    break;
                case CardinalDirection.West:
                    q = GeometryCompass.k_OneEightyYaw * q;
                    break;
                default:
                    throw new NotSupportedException();
            }

            return new Quaternion(q.z, -q.x, q.y, -q.w);
        }

        public static Quaternion ConvertToRUF(Quaternion q)
        {
            q = new Quaternion(-q.y, q.z, q.x, -q.w);
            switch (GeometryCompass.UnityZAxisDirection)
            {
                case CardinalDirection.North:
                    return GeometryCompass.k_NinetyYaw * q;
                case CardinalDirection.East:
                    return q;
                case CardinalDirection.South:
                    return GeometryCompass.k_NegativeNinetyYaw * q;
                case CardinalDirection.West:
                    return GeometryCompass.k_OneEightyYaw * q;
                default:
                    throw new NotSupportedException();
            }
        }

        Vector3 ICoordinateSpace.ConvertFromRUF(Vector3 v) => ENU.ConvertFromRUF(v);
        Vector3 ICoordinateSpace.ConvertToRUF(Vector3 v) => ENU.ConvertToRUF(v);
        Quaternion ICoordinateSpace.ConvertFromRUF(Quaternion q) => ENU.ConvertFromRUF(q);
        Quaternion ICoordinateSpace.ConvertToRUF(Quaternion q) => ENU.ConvertToRUF(q);
        //Note: Angular Velocity is the same as FLU / ENULocal
        Vector3 ICoordinateSpace.ConvertAngularVelocityFromRUF(Vector3 angularVelocity) => FLU.ConvertAngularVelocityFromRUF(angularVelocity);
        Vector3 ICoordinateSpace.ConvertAngularVelocityToRUF(Vector3 angularVelocity) => FLU.ConvertAngularVelocityToRUF(angularVelocity);
    }

    public enum CoordinateSpaceSelection
    {
        RUF,
        FLU,
        FRD,
        NED,
        ENU,
        NEDLocal,
        ENULocal,
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
                case CoordinateSpaceSelection.FRD:
                    return self.From<FRD>();
                case CoordinateSpaceSelection.ENU:
                    return self.From<ENU>();
                case CoordinateSpaceSelection.NED:
                    return self.From<NED>();
                case CoordinateSpaceSelection.ENULocal:
                    return self.From<ENULocal>();
                case CoordinateSpaceSelection.NEDLocal:
                    return self.From<NEDLocal>();
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
                case CoordinateSpaceSelection.FRD:
                    return self.From<FRD>();
                case CoordinateSpaceSelection.ENU:
                    return self.From<ENU>();
                case CoordinateSpaceSelection.NED:
                    return self.From<NED>();
                case CoordinateSpaceSelection.ENULocal:
                    return self.From<ENULocal>();
                case CoordinateSpaceSelection.NEDLocal:
                    return self.From<NEDLocal>();
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
                case CoordinateSpaceSelection.FRD:
                    return self.From<FRD>();
                case CoordinateSpaceSelection.ENU:
                    return self.From<ENU>();
                case CoordinateSpaceSelection.NED:
                    return self.From<NED>();
                case CoordinateSpaceSelection.ENULocal:
                    return self.From<ENULocal>();
                case CoordinateSpaceSelection.NEDLocal:
                    return self.From<NEDLocal>();
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
                case CoordinateSpaceSelection.FRD:
                    return self.From<FRD>();
                case CoordinateSpaceSelection.ENU:
                    return self.From<ENU>();
                case CoordinateSpaceSelection.NED:
                    return self.From<NED>();
                case CoordinateSpaceSelection.ENULocal:
                    return self.From<ENULocal>();
                case CoordinateSpaceSelection.NEDLocal:
                    return self.From<NEDLocal>();
                default:
                    Debug.LogError("Invalid coordinate space " + selection);
                    return self.From<RUF>();
            }
        }
    }
}
