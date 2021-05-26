using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Geometry
{
    public class MPose: Message {}
    public class MVector3 : Message {}
    public class MPoint : Message { }
    public class MPoint32 : Message { }
    public class MQuaternion : Message { }
    public class MTransform : Message { }
}

namespace Unity.Robotics.ROSTCPConnector.ROSGeometry
{
    public static class BackwardsCompatExtensions
    {
        public static Vector3<C> As<C>(this RosMessageTypes.Geometry.MPoint self) where C : ICoordinateSpace, new() => throw new NotImplementedException();

        public static Vector3 From<C>(this RosMessageTypes.Geometry.MPoint self) where C : ICoordinateSpace, new() => throw new NotImplementedException();

        public static Vector3<C> As<C>(this RosMessageTypes.Geometry.MPoint32 self) where C : ICoordinateSpace, new() => throw new NotImplementedException();

        public static Vector3 From<C>(this RosMessageTypes.Geometry.MPoint32 self) where C : ICoordinateSpace, new() => throw new NotImplementedException();

        public static Vector3<C> As<C>(this RosMessageTypes.Geometry.MVector3 self) where C : ICoordinateSpace, new() => throw new NotImplementedException();

        public static Vector3 From<C>(this RosMessageTypes.Geometry.MVector3 self) where C : ICoordinateSpace, new() => throw new NotImplementedException();

        public static Quaternion<C> As<C>(this RosMessageTypes.Geometry.MQuaternion self) where C : ICoordinateSpace, new() => throw new NotImplementedException();

        public static Quaternion From<C>(this RosMessageTypes.Geometry.MQuaternion self) where C : ICoordinateSpace, new() => throw new NotImplementedException();

        public static RosMessageTypes.Geometry.MTransform To<C>(this Transform transform) where C : ICoordinateSpace, new() => throw new NotImplementedException();

        public static Vector3 From(this RosMessageTypes.Geometry.MPoint self, CoordinateSpaceSelection selection) => throw new NotImplementedException();
        public static Vector3 From(this RosMessageTypes.Geometry.MPoint32 self, CoordinateSpaceSelection selection) => throw new NotImplementedException();
        public static Vector3 From(this RosMessageTypes.Geometry.Vector3Msg self, CoordinateSpaceSelection selection) => throw new NotImplementedException();
        public static Quaternion From(this RosMessageTypes.Geometry.MQuaternion self, CoordinateSpaceSelection selection) => throw new NotImplementedException();
    }
}