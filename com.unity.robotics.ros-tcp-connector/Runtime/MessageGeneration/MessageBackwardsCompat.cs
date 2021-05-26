using RosMessageTypes.BuiltinInterfaces;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Actionlib
{
    [Obsolete("MGoalID is now called GoalIDMsg")]
    public class MGoalID: GoalIDMsg {}
    [Obsolete("MGoalStatusArray is now called GoalStatusArrayMsg")]
    public class MGoalStatusArray: GoalStatusArrayMsg {}
    [Obsolete("MGoalStatus is now called GoalStatusMsg")]
    public class MGoalStatus: GoalStatusMsg {}
}

namespace RosMessageTypes.Diagnostic
{
    [Obsolete("MDiagnosticArray is now called DiagnosticArrayMsg")]
    public class MDiagnosticArray: DiagnosticArrayMsg {}
    [Obsolete("MDiagnosticStatus is now called DiagnosticStatusMsg")]
    public class MDiagnosticStatus: DiagnosticStatusMsg {}
    [Obsolete("MKeyValue is now called KeyValueMsg")]
    public class MKeyValue: KeyValueMsg {}
}

namespace RosMessageTypes.Geometry
{
    [Obsolete("MAccel is now called AccelMsg")]
    public class MAccel: AccelMsg {}
    [Obsolete("MAccelStamped is now called AccelStampedMsg")]
    public class MAccelStamped: AccelStampedMsg {}
    [Obsolete("MAccelWithCovariance is now called AccelWithCovarianceMsg")]
    public class MAccelWithCovariance: AccelWithCovarianceMsg {}
    [Obsolete("MAccelWithCovarianceStamped is now called AccelWithCovarianceStampedMsg")]
    public class MAccelWithCovarianceStamped: AccelWithCovarianceStampedMsg {}
    [Obsolete("MInertia is now called InertiaMsg")]
    public class MInertia: InertiaMsg {}
    [Obsolete("MInertiaStamped is now called InertiaStampedMsg")]
    public class MInertiaStamped: InertiaStampedMsg {}
    [Obsolete("MPoint32 is now called Point32Msg")]
    public class MPoint32: Point32Msg {}
    [Obsolete("MPoint is now called PointMsg")]
    public class MPoint: PointMsg {}
    [Obsolete("MPointStamped is now called PointStampedMsg")]
    public class MPointStamped: PointStampedMsg {}
    [Obsolete("MPolygon is now called PolygonMsg")]
    public class MPolygon: PolygonMsg {}
    [Obsolete("MPolygonStamped is now called PolygonStampedMsg")]
    public class MPolygonStamped: PolygonStampedMsg {}
    [Obsolete("MPose2D is now called Pose2DMsg")]
    public class MPose2D: Pose2DMsg {}
    [Obsolete("MPoseArray is now called PoseArrayMsg")]
    public class MPoseArray: PoseArrayMsg {}
    [Obsolete("MPose is now called PoseMsg")]
    public class MPose: PoseMsg {}
    [Obsolete("MPoseStamped is now called PoseStampedMsg")]
    public class MPoseStamped: PoseStampedMsg {}
    [Obsolete("MPoseWithCovariance is now called PoseWithCovarianceMsg")]
    public class MPoseWithCovariance: PoseWithCovarianceMsg {}
    [Obsolete("MPoseWithCovarianceStamped is now called PoseWithCovarianceStampedMsg")]
    public class MPoseWithCovarianceStamped: PoseWithCovarianceStampedMsg {}
    [Obsolete("MQuaternion is now called QuaternionMsg")]
    public class MQuaternion: QuaternionMsg {}
    [Obsolete("MQuaternionStamped is now called QuaternionStampedMsg")]
    public class MQuaternionStamped: QuaternionStampedMsg {}
    [Obsolete("MTransform is now called TransformMsg")]
    public class MTransform: TransformMsg {}
    [Obsolete("MTransformStamped is now called TransformStampedMsg")]
    public class MTransformStamped: TransformStampedMsg {}
    [Obsolete("MTwist is now called TwistMsg")]
    public class MTwist: TwistMsg {}
    [Obsolete("MTwistStamped is now called TwistStampedMsg")]
    public class MTwistStamped: TwistStampedMsg {}
    [Obsolete("MTwistWithCovariance is now called TwistWithCovarianceMsg")]
    public class MTwistWithCovariance: TwistWithCovarianceMsg {}
    [Obsolete("MTwistWithCovarianceStamped is now called TwistWithCovarianceStampedMsg")]
    public class MTwistWithCovarianceStamped: TwistWithCovarianceStampedMsg {}
    [Obsolete("MVector3 is now called Vector3Msg")]
    public class MVector3: Vector3Msg {}
    [Obsolete("MVector3Stamped is now called Vector3StampedMsg")]
    public class MVector3Stamped: Vector3StampedMsg {}
    [Obsolete("MWrench is now called WrenchMsg")]
    public class MWrench: WrenchMsg {}
    [Obsolete("MWrenchStamped is now called WrenchStampedMsg")]
    public class MWrenchStamped: WrenchStampedMsg {}
}

namespace RosMessageTypes.Nav
{
    [Obsolete("MGridCells is now called GridCellsMsg")]
    public class MGridCells: GridCellsMsg {}
    [Obsolete("MMapMetaData is now called MapMetaDataMsg")]
    public class MMapMetaData: MapMetaDataMsg {}
    [Obsolete("MOccupancyGrid is now called OccupancyGridMsg")]
    public class MOccupancyGrid: OccupancyGridMsg {}
    [Obsolete("MOdometry is now called OdometryMsg")]
    public class MOdometry: OdometryMsg {}
    [Obsolete("MPath is now called PathMsg")]
    public class MPath: PathMsg {}
}

namespace RosMessageTypes.Sensor
{
    [Obsolete("MBatteryState is now called BatteryStateMsg")]
    public class MBatteryState: BatteryStateMsg {}
    [Obsolete("MCameraInfo is now called CameraInfoMsg")]
    public class MCameraInfo: CameraInfoMsg {}
    [Obsolete("MChannelFloat32 is now called ChannelFloat32Msg")]
    public class MChannelFloat32: ChannelFloat32Msg {}
    [Obsolete("MCompressedImage is now called CompressedImageMsg")]
    public class MCompressedImage: CompressedImageMsg {}
    [Obsolete("MFluidPressure is now called FluidPressureMsg")]
    public class MFluidPressure: FluidPressureMsg {}
    [Obsolete("MIlluminance is now called IlluminanceMsg")]
    public class MIlluminance: IlluminanceMsg {}
    [Obsolete("MImage is now called ImageMsg")]
    public class MImage: ImageMsg {}
    [Obsolete("MImu is now called ImuMsg")]
    public class MImu: ImuMsg {}
    [Obsolete("MJointState is now called JointStateMsg")]
    public class MJointState: JointStateMsg {}
    [Obsolete("MJoyFeedbackArray is now called JoyFeedbackArrayMsg")]
    public class MJoyFeedbackArray: JoyFeedbackArrayMsg {}
    [Obsolete("MJoyFeedback is now called JoyFeedbackMsg")]
    public class MJoyFeedback: JoyFeedbackMsg {}
    [Obsolete("MJoy is now called JoyMsg")]
    public class MJoy: JoyMsg {}
    [Obsolete("MLaserEcho is now called LaserEchoMsg")]
    public class MLaserEcho: LaserEchoMsg {}
    [Obsolete("MLaserScan is now called LaserScanMsg")]
    public class MLaserScan: LaserScanMsg {}
    [Obsolete("MMagneticField is now called MagneticFieldMsg")]
    public class MMagneticField: MagneticFieldMsg {}
    [Obsolete("MMultiDOFJointState is now called MultiDOFJointStateMsg")]
    public class MMultiDOFJointState: MultiDOFJointStateMsg {}
    [Obsolete("MMultiEchoLaserScan is now called MultiEchoLaserScanMsg")]
    public class MMultiEchoLaserScan: MultiEchoLaserScanMsg {}
    [Obsolete("MNavSatFix is now called NavSatFixMsg")]
    public class MNavSatFix: NavSatFixMsg {}
    [Obsolete("MNavSatStatus is now called NavSatStatusMsg")]
    public class MNavSatStatus: NavSatStatusMsg {}
    [Obsolete("MPointCloud2 is now called PointCloud2Msg")]
    public class MPointCloud2: PointCloud2Msg {}
    [Obsolete("MPointCloud is now called PointCloudMsg")]
    public class MPointCloud: PointCloudMsg {}
    [Obsolete("MPointField is now called PointFieldMsg")]
    public class MPointField: PointFieldMsg {}
    [Obsolete("MRange is now called RangeMsg")]
    public class MRange: RangeMsg {}
    [Obsolete("MRegionOfInterest is now called RegionOfInterestMsg")]
    public class MRegionOfInterest: RegionOfInterestMsg {}
    [Obsolete("MRelativeHumidity is now called RelativeHumidityMsg")]
    public class MRelativeHumidity: RelativeHumidityMsg {}
    [Obsolete("MTemperature is now called TemperatureMsg")]
    public class MTemperature: TemperatureMsg {}
    [Obsolete("MTimeReference is now called TimeReferenceMsg")]
    public class MTimeReference: TimeReferenceMsg {}
}

namespace RosMessageTypes.Shape
{
    [Obsolete("MMesh is now called MeshMsg")]
    public class MMesh: MeshMsg {}
    [Obsolete("MMeshTriangle is now called MeshTriangleMsg")]
    public class MMeshTriangle: MeshTriangleMsg {}
    [Obsolete("MPlane is now called PlaneMsg")]
    public class MPlane: PlaneMsg {}
    [Obsolete("MSolidPrimitive is now called SolidPrimitiveMsg")]
    public class MSolidPrimitive: SolidPrimitiveMsg {}
}

namespace RosMessageTypes.Std
{
    [Obsolete("MBool is now called BoolMsg")]
    public class MBool: BoolMsg {}
    [Obsolete("MByte is now called ByteMsg")]
    public class MByte: ByteMsg {}
    [Obsolete("MByteMultiArray is now called ByteMultiArrayMsg")]
    public class MByteMultiArray: ByteMultiArrayMsg {}
    [Obsolete("MChar is now called CharMsg")]
    public class MChar: CharMsg {}
    [Obsolete("MColorRGBA is now called ColorRGBAMsg")]
    public class MColorRGBA: ColorRGBAMsg {}
    [Obsolete("MEmpty is now called EmptyMsg")]
    public class MEmpty: EmptyMsg {}
    [Obsolete("MFloat32 is now called Float32Msg")]
    public class MFloat32: Float32Msg {}
    [Obsolete("MFloat32MultiArray is now called Float32MultiArrayMsg")]
    public class MFloat32MultiArray: Float32MultiArrayMsg {}
    [Obsolete("MFloat64 is now called Float64Msg")]
    public class MFloat64: Float64Msg {}
    [Obsolete("MFloat64MultiArray is now called Float64MultiArrayMsg")]
    public class MFloat64MultiArray: Float64MultiArrayMsg {}
    [Obsolete("MInt16 is now called Int16Msg")]
    public class MInt16: Int16Msg {}
    [Obsolete("MInt16MultiArray is now called Int16MultiArrayMsg")]
    public class MInt16MultiArray: Int16MultiArrayMsg {}
    [Obsolete("MInt32 is now called Int32Msg")]
    public class MInt32: Int32Msg {}
    [Obsolete("MInt32MultiArray is now called Int32MultiArrayMsg")]
    public class MInt32MultiArray: Int32MultiArrayMsg {}
    [Obsolete("MInt64 is now called Int64Msg")]
    public class MInt64: Int64Msg {}
    [Obsolete("MInt64MultiArray is now called Int64MultiArrayMsg")]
    public class MInt64MultiArray: Int64MultiArrayMsg {}
    [Obsolete("MInt8 is now called Int8Msg")]
    public class MInt8: Int8Msg {}
    [Obsolete("MInt8MultiArray is now called Int8MultiArrayMsg")]
    public class MInt8MultiArray: Int8MultiArrayMsg {}
    [Obsolete("MMultiArrayDimension is now called MultiArrayDimensionMsg")]
    public class MMultiArrayDimension: MultiArrayDimensionMsg {}
    [Obsolete("MMultiArrayLayout is now called MultiArrayLayoutMsg")]
    public class MMultiArrayLayout: MultiArrayLayoutMsg {}
    [Obsolete("MString is now called StringMsg")]
    public class MString: StringMsg {}
    [Obsolete("MUInt16 is now called UInt16Msg")]
    public class MUInt16: UInt16Msg {}
    [Obsolete("MUInt16MultiArray is now called UInt16MultiArrayMsg")]
    public class MUInt16MultiArray: UInt16MultiArrayMsg {}
    [Obsolete("MUInt32 is now called UInt32Msg")]
    public class MUInt32: UInt32Msg {}
    [Obsolete("MUInt32MultiArray is now called UInt32MultiArrayMsg")]
    public class MUInt32MultiArray: UInt32MultiArrayMsg {}
    [Obsolete("MUInt64 is now called UInt64Msg")]
    public class MUInt64: UInt64Msg {}
    [Obsolete("MUInt64MultiArray is now called UInt64MultiArrayMsg")]
    public class MUInt64MultiArray: UInt64MultiArrayMsg {}
    [Obsolete("MUInt8 is now called UInt8Msg")]
    public class MUInt8: UInt8Msg {}
    [Obsolete("MUInt8MultiArray is now called UInt8MultiArrayMsg")]
    public class MUInt8MultiArray: UInt8MultiArrayMsg {}
    [Obsolete("MHeader is now called HeaderMsg")]
    public class MHeader: HeaderMsg {}
    [Obsolete("MTime is now called TimeMsg")]
    public class MTime: TimeMsg {}
    [Obsolete("MDuration is now called DurationMsg")]
    public class MDuration: DurationMsg {}
}

namespace RosMessageTypes.Stereo
{
    [Obsolete("MDisparityImage is now called DisparityImageMsg")]
    public class MDisparityImage: DisparityImageMsg {}
}

namespace RosMessageTypes.Trajectory
{
    [Obsolete("MJointTrajectory is now called JointTrajectoryMsg")]
    public class MJointTrajectory: JointTrajectoryMsg {}
    [Obsolete("MJointTrajectoryPoint is now called JointTrajectoryPointMsg")]
    public class MJointTrajectoryPoint: JointTrajectoryPointMsg {}
    [Obsolete("MMultiDOFJointTrajectory is now called MultiDOFJointTrajectoryMsg")]
    public class MMultiDOFJointTrajectory: MultiDOFJointTrajectoryMsg {}
    [Obsolete("MMultiDOFJointTrajectoryPoint is now called MultiDOFJointTrajectoryPointMsg")]
    public class MMultiDOFJointTrajectoryPoint: MultiDOFJointTrajectoryPointMsg {}
}

namespace RosMessageTypes.ObjectRecognition
{
    [Obsolete("MObjectInformation is now called ObjectInformationMsg")]
    public class MObjectInformation: ObjectInformationMsg {}
    [Obsolete("MObjectType is now called ObjectTypeMsg")]
    public class MObjectType: ObjectTypeMsg {}
    [Obsolete("MRecognizedObjectArray is now called RecognizedObjectArrayMsg")]
    public class MRecognizedObjectArray: RecognizedObjectArrayMsg {}
    [Obsolete("MRecognizedObject is now called RecognizedObjectMsg")]
    public class MRecognizedObject: RecognizedObjectMsg {}
    [Obsolete("MTableArray is now called TableArrayMsg")]
    public class MTableArray: TableArrayMsg {}
    public class MTable: TableMsg { }
}

namespace RosMessageTypes.Octomap
{
    [Obsolete("MOctomap is now called OctomapMsg")]
    public class MOctomap: OctomapMsg {}
    [Obsolete("MOctomapWithPose is now called OctomapWithPoseMsg")]
    public class MOctomapWithPose: OctomapWithPoseMsg {}
}