using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Diagnostic
{
    public class MDiagnosticArray : Message { }
    public class MDiagnosticStatus : Message { }
    public class MKeyValue : Message { }
}

namespace RosMessageTypes.Geometry
{
    public class MAccel : Message { }
    public class MAccelStamped : Message { }
    public class MAccelWithCovariance : Message { }
    public class MAccelWithCovarianceStamped : Message { }
    public class MInertia : Message { }
    public class MInertiaStamped : Message { }
    public class MPoint32 : Message { }
    public class MPoint : Message { }
    public class MPointStamped : Message { }
    public class MPolygon : Message { }
    public class MPolygonStamped : Message { }
    public class MPose2D : Message { }
    public class MPoseArray : Message { }
    public class MPose : Message { }
    public class MPoseStamped : Message { }
    public class MPoseWithCovariance : Message { }
    public class MPoseWithCovarianceStamped : Message { }
    public class MQuaternion : Message { }
    public class MQuaternionStamped : Message { }
    public class MTransform : Message { }
    public class MTransformStamped : Message { }
    public class MTwist : Message { }
    public class MTwistStamped : Message { }
    public class MTwistWithCovariance : Message { }
    public class MTwistWithCovarianceStamped : Message { }
    public class MVector3 : Message { }
    public class MVector3Stamped : Message { }
    public class MWrench : Message { }
    public class MWrenchStamped : Message { }
}

namespace RosMessageTypes.Nav
{
    public class MGridCells : Message { }
    public class MMapMetaData : Message { }
    public class MOccupancyGrid : Message { }
    public class MOdometry : Message { }
    public class MPath : Message { }
}

namespace RosMessageTypes.Sensor
{
    public class MBatteryState : Message { }
    public class MCameraInfo : Message { }
    public class MChannelFloat32 : Message { }
    public class MCompressedImage : Message { }
    public class MFluidPressure : Message { }
    public class MIlluminance : Message { }
    public class MImage : Message { }
    public class MImu : Message { }
    public class MJointState : Message { }
    public class MJoyFeedbackArray : Message { }
    public class MJoyFeedback : Message { }
    public class MJoy : Message { }
    public class MLaserEcho : Message { }
    public class MLaserScan : Message { }
    public class MMagneticField : Message { }
    public class MMultiDOFJointState : Message { }
    public class MMultiEchoLaserScan : Message { }
    public class MNavSatFix : Message { }
    public class MNavSatStatus : Message { }
    public class MPointCloud2 : Message { }
    public class MPointCloud : Message { }
    public class MPointField : Message { }
    public class MRange : Message { }
    public class MRegionOfInterest : Message { }
    public class MRelativeHumidity : Message { }
    public class MTemperature : Message { }
    public class MTimeReference : Message { }
}

namespace RosMessageTypes.Shape
{
    public class MMesh : Message { }
    public class MMeshTriangle : Message { }
    public class MPlane : Message { }
    public class MSolidPrimitive : Message { }
}

namespace RosMessageTypes.Std
{
    public class MBool : Message { }
    public class MByte : Message { }
    public class MByteMultiArray : Message { }
    public class MChar : Message { }
    public class MColorRGBA : Message { }
    public class MEmpty : Message { }
    public class MFloat32 : Message { }
    public class MFloat32MultiArray : Message { }
    public class MFloat64 : Message { }
    public class MFloat64MultiArray : Message { }
    public class MInt16 : Message { }
    public class MInt16MultiArray : Message { }
    public class MInt32 : Message { }
    public class MInt32MultiArray : Message { }
    public class MInt64 : Message { }
    public class MInt64MultiArray : Message { }
    public class MInt8 : Message { }
    public class MInt8MultiArray : Message { }
    public class MMultiArrayDimension : Message { }
    public class MMultiArrayLayout : Message { }
    public class MString : Message { }
    public class MUInt16 : Message { }
    public class MUInt16MultiArray : Message { }
    public class MUInt32 : Message { }
    public class MUInt32MultiArray : Message { }
    public class MUInt64 : Message { }
    public class MUInt64MultiArray : Message { }
    public class MUInt8 : Message { }
    public class MUInt8MultiArray : Message { }
    public class MHeader : Message { }
    public class MTime : Message { }
    public class MDuration : Message { }
}

namespace RosMessageTypes.Stereo
{
    public class MDisparityImage : Message { }
}

namespace RosMessageTypes.Trajectory
{
    public class MJointTrajectory : Message { }
    public class MJointTrajectoryPoint : Message { }
    public class MMultiDOFJointTrajectory : Message { }
    public class MMultiDOFJointTrajectoryPoint : Message { }
}