using RosMessageTypes.BuiltinInterfaces;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Diagnostic
{
    public class MDiagnosticArray : DiagnosticArrayMsg { }
    public class MDiagnosticStatus : DiagnosticStatusMsg { }
    public class MKeyValue : KeyValueMsg { }
}

namespace RosMessageTypes.Geometry
{
    public class MAccel : AccelMsg { }
    public class MAccelStamped : AccelStampedMsg { }
    public class MAccelWithCovariance : AccelWithCovarianceMsg { }
    public class MAccelWithCovarianceStamped : AccelWithCovarianceStampedMsg { }
    public class MInertia : InertiaMsg { }
    public class MInertiaStamped : InertiaStampedMsg { }
    public class MPoint32 : Point32Msg { }
    public class MPoint : PointMsg { }
    public class MPointStamped : PointStampedMsg { }
    public class MPolygon : PolygonMsg { }
    public class MPolygonStamped : PolygonStampedMsg { }
    public class MPose2D : Pose2DMsg { }
    public class MPoseArray : PoseArrayMsg { }
    public class MPose : PoseMsg { }
    public class MPoseStamped : PoseStampedMsg { }
    public class MPoseWithCovariance : PoseWithCovarianceMsg { }
    public class MPoseWithCovarianceStamped : PoseWithCovarianceStampedMsg { }
    public class MQuaternion : QuaternionMsg { }
    public class MQuaternionStamped : QuaternionStampedMsg { }
    public class MTransform : TransformMsg { }
    public class MTransformStamped : TransformStampedMsg { }
    public class MTwist : TwistMsg { }
    public class MTwistStamped : TwistStampedMsg { }
    public class MTwistWithCovariance : TwistWithCovarianceMsg { }
    public class MTwistWithCovarianceStamped : TwistWithCovarianceStampedMsg { }
    public class MVector3 : Vector3Msg { }
    public class MVector3Stamped : Vector3StampedMsg { }
    public class MWrench : WrenchMsg { }
    public class MWrenchStamped : WrenchStampedMsg { }
}

namespace RosMessageTypes.Nav
{
    public class MGridCells : GridCellsMsg { }
    public class MMapMetaData : MapMetaDataMsg { }
    public class MOccupancyGrid : OccupancyGridMsg { }
    public class MOdometry : OdometryMsg { }
    public class MPath : PathMsg { }
}

namespace RosMessageTypes.Sensor
{
    public class MBatteryState : BatteryStateMsg { }
    public class MCameraInfo : CameraInfoMsg { }
    public class MChannelFloat32 : ChannelFloat32Msg { }
    public class MCompressedImage : CompressedImageMsg { }
    public class MFluidPressure : FluidPressureMsg { }
    public class MIlluminance : IlluminanceMsg { }
    public class MImage : ImageMsg { }
    public class MImu : ImuMsg { }
    public class MJointState : JointStateMsg { }
    public class MJoyFeedbackArray : JoyFeedbackArrayMsg { }
    public class MJoyFeedback : JoyFeedbackMsg { }
    public class MJoy : JoyMsg { }
    public class MLaserEcho : LaserEchoMsg { }
    public class MLaserScan : LaserScanMsg { }
    public class MMagneticField : MagneticFieldMsg { }
    public class MMultiDOFJointState : MultiDOFJointStateMsg { }
    public class MMultiEchoLaserScan : MultiEchoLaserScanMsg { }
    public class MNavSatFix : NavSatFixMsg { }
    public class MNavSatStatus : NavSatStatusMsg { }
    public class MPointCloud2 : PointCloud2Msg { }
    public class MPointCloud : PointCloudMsg { }
    public class MPointField : PointFieldMsg { }
    public class MRange : RangeMsg { }
    public class MRegionOfInterest : RegionOfInterestMsg { }
    public class MRelativeHumidity : RelativeHumidityMsg { }
    public class MTemperature : TemperatureMsg { }
    public class MTimeReference : TimeReferenceMsg { }
}

namespace RosMessageTypes.Shape
{
    public class MMesh : MeshMsg { }
    public class MMeshTriangle : MeshTriangleMsg { }
    public class MPlane : PlaneMsg { }
    public class MSolidPrimitive : SolidPrimitiveMsg { }
}

namespace RosMessageTypes.Std
{
    public class MBool : BoolMsg { }
    public class MByte : ByteMsg { }
    public class MByteMultiArray : ByteMultiArrayMsg { }
    public class MChar : CharMsg { }
    public class MColorRGBA : ColorRGBAMsg { }
    public class MEmpty : EmptyMsg { }
    public class MFloat32 : Float32Msg { }
    public class MFloat32MultiArray : Float32MultiArrayMsg { }
    public class MFloat64 : Float64Msg { }
    public class MFloat64MultiArray : Float64MultiArrayMsg { }
    public class MInt16 : Int16Msg { }
    public class MInt16MultiArray : Int16MultiArrayMsg { }
    public class MInt32 : Int32Msg { }
    public class MInt32MultiArray : Int32MultiArrayMsg { }
    public class MInt64 : Int64Msg { }
    public class MInt64MultiArray : Int64MultiArrayMsg { }
    public class MInt8 : Int8Msg { }
    public class MInt8MultiArray : Int8MultiArrayMsg { }
    public class MMultiArrayDimension : MultiArrayDimensionMsg { }
    public class MMultiArrayLayout : MultiArrayLayoutMsg { }
    public class MString : StringMsg { }
    public class MUInt16 : UInt16Msg { }
    public class MUInt16MultiArray : UInt16MultiArrayMsg { }
    public class MUInt32 : UInt32Msg { }
    public class MUInt32MultiArray : UInt32MultiArrayMsg { }
    public class MUInt64 : UInt64Msg { }
    public class MUInt64MultiArray : UInt64MultiArrayMsg { }
    public class MUInt8 : UInt8Msg { }
    public class MUInt8MultiArray : UInt8MultiArrayMsg { }
    public class MHeader : HeaderMsg { }
    public class MTime : TimeMsg { }
    public class MDuration : DurationMsg { }
}

namespace RosMessageTypes.Stereo
{
    public class MDisparityImage : DisparityImageMsg { }
}

namespace RosMessageTypes.Trajectory
{
    public class MJointTrajectory : JointTrajectoryMsg { }
    public class MJointTrajectoryPoint : JointTrajectoryPointMsg { }
    public class MMultiDOFJointTrajectory : MultiDOFJointTrajectoryMsg { }
    public class MMultiDOFJointTrajectoryPoint : MultiDOFJointTrajectoryPointMsg { }
}

namespace RosMessageTypes.ObjectRecognition
{
    public class MObjectInformation : ObjectInformationMsg { }
    public class MObjectType : ObjectTypeMsg { }
    public class MRecognizedObjectArray : RecognizedObjectArrayMsg { }
    public class MRecognizedObject : RecognizedObjectMsg { }
    public class MTableArray : TableArrayMsg { }
    public class MTable: TableMsg { }
}

namespace RosMessageTypes.Octomap
{
    public class MOctomap : OctomapMsg { }
    public class MOctomapWithPose : OctomapWithPoseMsg { }
}