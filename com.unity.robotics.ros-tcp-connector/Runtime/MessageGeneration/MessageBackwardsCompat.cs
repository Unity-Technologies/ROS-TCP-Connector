using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System;
using UnityEngine;

namespace RosMessageTypes.Actionlib
{
    [Obsolete("MGoalID is now called GoalIDMsg")]
    public class MGoalID : GoalIDMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MGoalID() { }
        public MGoalID(MTime stamp, string id) { }
    }

    [Obsolete("MGoalStatus is now called GoalStatusMsg")]
    public class MGoalStatus : GoalStatusMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MGoalStatus() { }
        public MGoalStatus(MGoalID goal_id, byte status, string text) { }
    }

    [Obsolete("MGoalStatusArray is now called GoalStatusArrayMsg")]
    public class MGoalStatusArray : GoalStatusArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MGoalStatusArray() { }
        public MGoalStatusArray(MHeader header, MGoalStatus[] status_list) { }
    }
}

namespace RosMessageTypes.Diagnostic
{
    [Obsolete("MDiagnosticArray is now called DiagnosticArrayMsg")]
    public class MDiagnosticArray : DiagnosticArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MDiagnosticArray() { }
        public MDiagnosticArray(MHeader header, MDiagnosticStatus[] status) { }
    }

    [Obsolete("MDiagnosticStatus is now called DiagnosticStatusMsg")]
    public class MDiagnosticStatus : DiagnosticStatusMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MDiagnosticStatus() { }
        public MDiagnosticStatus(sbyte level, string name, string message, string hardware_id, MKeyValue[] values) { }
    }

    [Obsolete("MKeyValue is now called KeyValueMsg")]
    public class MKeyValue : KeyValueMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MKeyValue() { }
        public MKeyValue(string key, string value) { }
    }

    [Obsolete("MAddDiagnosticsRequest is now called AddDiagnosticsRequestMsg")]
    public class MAddDiagnosticsRequest : AddDiagnosticsRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MAddDiagnosticsRequest() { }
        public MAddDiagnosticsRequest(string load_namespace) { }
    }

    [Obsolete("MAddDiagnosticsResponse is now called AddDiagnosticsResponseMsg")]
    public class MAddDiagnosticsResponse : AddDiagnosticsResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MAddDiagnosticsResponse() { }
        public MAddDiagnosticsResponse(bool success, string message) { }
    }

    [Obsolete("MSelfTestRequest is now called SelfTestRequestMsg")]
    public class MSelfTestRequest : SelfTestRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MSelfTestRequest() { }
    }

    [Obsolete("MSelfTestResponse is now called SelfTestResponseMsg")]
    public class MSelfTestResponse : SelfTestResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MSelfTestResponse() { }
        public MSelfTestResponse(string id, sbyte passed, MDiagnosticStatus[] status) { }
    }
}

namespace RosMessageTypes.Geometry
{
    [Obsolete("MAccel is now called AccelMsg")]
    public class MAccel : AccelMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MAccel() { }
        public MAccel(MVector3 linear, MVector3 angular) { }
    }

    [Obsolete("MAccelStamped is now called AccelStampedMsg")]
    public class MAccelStamped : AccelStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MAccelStamped() { }
        public MAccelStamped(MHeader header, MAccel accel) { }
    }

    [Obsolete("MAccelWithCovariance is now called AccelWithCovarianceMsg")]
    public class MAccelWithCovariance : AccelWithCovarianceMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MAccelWithCovariance() { }
        public MAccelWithCovariance(MAccel accel, double[] covariance) { }
    }

    [Obsolete("MAccelWithCovarianceStamped is now called AccelWithCovarianceStampedMsg")]
    public class MAccelWithCovarianceStamped : AccelWithCovarianceStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MAccelWithCovarianceStamped() { }
        public MAccelWithCovarianceStamped(MHeader header, MAccelWithCovariance accel) { }
    }

    [Obsolete("MInertia is now called InertiaMsg")]
    public class MInertia : InertiaMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInertia() { }
        public MInertia(double m, MVector3 com, double ixx, double ixy, double ixz, double iyy, double iyz, double izz) { }
    }

    [Obsolete("MInertiaStamped is now called InertiaStampedMsg")]
    public class MInertiaStamped : InertiaStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInertiaStamped() { }
        public MInertiaStamped(MHeader header, MInertia inertia) { }
    }

    [Obsolete("MPoint is now called PointMsg")]
    public class MPoint : PointMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPoint() { }
        public MPoint(double x, double y, double z) { }
    }

    [Obsolete("MPoint32 is now called Point32Msg")]
    public class MPoint32 : Point32Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPoint32() { }
        public MPoint32(float x, float y, float z) { }
    }

    [Obsolete("MPointStamped is now called PointStampedMsg")]
    public class MPointStamped : PointStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPointStamped() { }
        public MPointStamped(MHeader header, MPoint point) { }
    }

    [Obsolete("MPolygon is now called PolygonMsg")]
    public class MPolygon : PolygonMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPolygon() { }
        public MPolygon(MPoint32[] points) { }
    }

    [Obsolete("MPolygonStamped is now called PolygonStampedMsg")]
    public class MPolygonStamped : PolygonStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPolygonStamped() { }
        public MPolygonStamped(MHeader header, MPolygon polygon) { }
    }

    [Obsolete("MPose is now called PoseMsg")]
    public class MPose : PoseMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPose() { }
        public MPose(MPoint position, MQuaternion orientation) { }
    }

    [Obsolete("MPose2D is now called Pose2DMsg")]
    public class MPose2D : Pose2DMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPose2D() { }
        public MPose2D(double x, double y, double theta) { }
    }

    [Obsolete("MPoseArray is now called PoseArrayMsg")]
    public class MPoseArray : PoseArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPoseArray() { }
        public MPoseArray(MHeader header, MPose[] poses) { }
    }

    [Obsolete("MPoseStamped is now called PoseStampedMsg")]
    public class MPoseStamped : PoseStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPoseStamped() { }
        public MPoseStamped(MHeader header, MPose pose) { }
    }

    [Obsolete("MPoseWithCovariance is now called PoseWithCovarianceMsg")]
    public class MPoseWithCovariance : PoseWithCovarianceMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPoseWithCovariance() { }
        public MPoseWithCovariance(MPose pose, double[] covariance) { }
    }

    [Obsolete("MPoseWithCovarianceStamped is now called PoseWithCovarianceStampedMsg")]
    public class MPoseWithCovarianceStamped : PoseWithCovarianceStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPoseWithCovarianceStamped() { }
        public MPoseWithCovarianceStamped(MHeader header, MPoseWithCovariance pose) { }
    }

    [Obsolete("MQuaternion is now called QuaternionMsg")]
    public class MQuaternion : QuaternionMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MQuaternion() { }
        public MQuaternion(double x, double y, double z, double w) { }
    }

    [Obsolete("MQuaternionStamped is now called QuaternionStampedMsg")]
    public class MQuaternionStamped : QuaternionStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MQuaternionStamped() { }
        public MQuaternionStamped(MHeader header, MQuaternion quaternion) { }
    }

    [Obsolete("MTransform is now called TransformMsg")]
    public class MTransform : TransformMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTransform() { }
        public MTransform(MVector3 translation, MQuaternion rotation) { }
    }

    [Obsolete("MTransformStamped is now called TransformStampedMsg")]
    public class MTransformStamped : TransformStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTransformStamped() { }
        public MTransformStamped(MHeader header, string child_frame_id, MTransform transform) { }
    }

    [Obsolete("MTwist is now called TwistMsg")]
    public class MTwist : TwistMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTwist() { }
        public MTwist(MVector3 linear, MVector3 angular) { }
    }

    [Obsolete("MTwistStamped is now called TwistStampedMsg")]
    public class MTwistStamped : TwistStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTwistStamped() { }
        public MTwistStamped(MHeader header, MTwist twist) { }
    }

    [Obsolete("MTwistWithCovariance is now called TwistWithCovarianceMsg")]
    public class MTwistWithCovariance : TwistWithCovarianceMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTwistWithCovariance() { }
        public MTwistWithCovariance(MTwist twist, double[] covariance) { }
    }

    [Obsolete("MTwistWithCovarianceStamped is now called TwistWithCovarianceStampedMsg")]
    public class MTwistWithCovarianceStamped : TwistWithCovarianceStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTwistWithCovarianceStamped() { }
        public MTwistWithCovarianceStamped(MHeader header, MTwistWithCovariance twist) { }
    }

    [Obsolete("MVector3 is now called Vector3Msg")]
    public class MVector3 : Vector3Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MVector3() { }
        public MVector3(double x, double y, double z) { }
    }

    [Obsolete("MVector3Stamped is now called Vector3StampedMsg")]
    public class MVector3Stamped : Vector3StampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MVector3Stamped() { }
        public MVector3Stamped(MHeader header, MVector3 vector) { }
    }

    [Obsolete("MWrench is now called WrenchMsg")]
    public class MWrench : WrenchMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MWrench() { }
        public MWrench(MVector3 force, MVector3 torque) { }
    }

    [Obsolete("MWrenchStamped is now called WrenchStampedMsg")]
    public class MWrenchStamped : WrenchStampedMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MWrenchStamped() { }
        public MWrenchStamped(MHeader header, MWrench wrench) { }
    }
}

namespace RosMessageTypes.Nav
{
    [Obsolete("MGetMapFeedback is now called GetMapFeedbackMsg")]
    public class MGetMapFeedback : GetMapFeedback
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetMapFeedback() { }
    }

    [Obsolete("MGetMapGoal is now called GetMapGoalMsg")]
    public class MGetMapGoal : GetMapGoal
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetMapGoal() { }
    }

    [Obsolete("MGetMapResult is now called GetMapResultMsg")]
    public class MGetMapResult : GetMapResult
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetMapResult() { }
        public MGetMapResult(MOccupancyGrid map) { }
    }

    [Obsolete("MGridCells is now called GridCellsMsg")]
    public class MGridCells : GridCellsMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MGridCells() { }
        public MGridCells(MHeader header, float cell_width, float cell_height, Geometry.MPoint[] cells) { }
    }

    [Obsolete("MMapMetaData is now called MapMetaDataMsg")]
    public class MMapMetaData : MapMetaDataMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMapMetaData() { }
        public MMapMetaData(MTime map_load_time, float resolution, uint width, uint height, Geometry.MPose origin) { }
    }

    [Obsolete("MOccupancyGrid is now called OccupancyGridMsg")]
    public class MOccupancyGrid : OccupancyGridMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MOccupancyGrid() { }
        public MOccupancyGrid(MHeader header, MMapMetaData info, sbyte[] data) { }
    }

    [Obsolete("MOdometry is now called OdometryMsg")]
    public class MOdometry : OdometryMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MOdometry() { }
        public MOdometry(MHeader header, string child_frame_id, Geometry.MPoseWithCovariance pose, Geometry.MTwistWithCovariance twist) { }
    }

    [Obsolete("MPath is now called PathMsg")]
    public class MPath : PathMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPath() { }
        public MPath(MHeader header, Geometry.MPoseStamped[] poses) { }
    }

    [Obsolete("MGetMapRequest is now called GetMapRequestMsg")]
    public class MGetMapRequest : GetMapRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetMapRequest() { }
    }

    [Obsolete("MGetMapResponse is now called GetMapResponseMsg")]
    public class MGetMapResponse : GetMapResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetMapResponse() { }
        public MGetMapResponse(MOccupancyGrid map) { }
    }

    [Obsolete("MGetPlanRequest is now called GetPlanRequestMsg")]
    public class MGetPlanRequest : GetPlanRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetPlanRequest() { }
        public MGetPlanRequest(Geometry.MPoseStamped start, Geometry.MPoseStamped goal, float tolerance) { }
    }

    [Obsolete("MGetPlanResponse is now called GetPlanResponseMsg")]
    public class MGetPlanResponse : GetPlanResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetPlanResponse() { }
        public MGetPlanResponse(MPath plan) { }
    }

    [Obsolete("MLoadMapRequest is now called LoadMapRequestMsg")]
    public class MLoadMapRequest : LoadMapRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MLoadMapRequest() { }
        public MLoadMapRequest(string map_url) { }
    }

    [Obsolete("MLoadMapResponse is now called LoadMapResponseMsg")]
    public class MLoadMapResponse : LoadMapResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MLoadMapResponse() { }
        public MLoadMapResponse(MOccupancyGrid map, byte result) { }
    }

    [Obsolete("MSetMapRequest is now called SetMapRequestMsg")]
    public class MSetMapRequest : SetMapRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MSetMapRequest() { }
        public MSetMapRequest(MOccupancyGrid map, Geometry.MPoseWithCovarianceStamped initial_pose) { }
    }

    [Obsolete("MSetMapResponse is now called SetMapResponseMsg")]
    public class MSetMapResponse : SetMapResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MSetMapResponse() { }
        public MSetMapResponse(bool success) { }
    }
}

namespace RosMessageTypes.ObjectRecognition
{
    [Obsolete("MObjectRecognitionFeedback is now called ObjectRecognitionFeedbackMsg")]
    public class MObjectRecognitionFeedback : ObjectRecognitionFeedback
    {
        public const string RosMessageName = k_RosMessageName;
        public MObjectRecognitionFeedback() { }
    }

    [Obsolete("MObjectRecognitionGoal is now called ObjectRecognitionGoalMsg")]
    public class MObjectRecognitionGoal : ObjectRecognitionGoal
    {
        public const string RosMessageName = k_RosMessageName;
        public MObjectRecognitionGoal() { }
        public MObjectRecognitionGoal(bool use_roi, float[] filter_limits) { }
    }

    [Obsolete("MObjectRecognitionResult is now called ObjectRecognitionResultMsg")]
    public class MObjectRecognitionResult : ObjectRecognitionResult
    {
        public const string RosMessageName = k_RosMessageName;
        public MObjectRecognitionResult() { }
        public MObjectRecognitionResult(MRecognizedObjectArray recognized_objects) { }
    }

    [Obsolete("MObjectInformation is now called ObjectInformationMsg")]
    public class MObjectInformation : ObjectInformationMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MObjectInformation() { }
        public MObjectInformation(string name, Shape.MMesh ground_truth_mesh, Sensor.MPointCloud2 ground_truth_point_cloud) { }
    }

    [Obsolete("MObjectType is now called ObjectTypeMsg")]
    public class MObjectType : ObjectTypeMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MObjectType() { }
        public MObjectType(string key, string db) { }
    }

    [Obsolete("MRecognizedObject is now called RecognizedObjectMsg")]
    public class MRecognizedObject : RecognizedObjectMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MRecognizedObject() { }
        public MRecognizedObject(MHeader header, MObjectType type, float confidence, Sensor.MPointCloud2[] point_clouds, Shape.MMesh bounding_mesh, Geometry.MPoint[] bounding_contours, Geometry.MPoseWithCovarianceStamped pose) { }
    }

    [Obsolete("MRecognizedObjectArray is now called RecognizedObjectArrayMsg")]
    public class MRecognizedObjectArray : RecognizedObjectArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MRecognizedObjectArray() { }
        public MRecognizedObjectArray(MHeader header, MRecognizedObject[] objects, float[] cooccurrence) { }
    }

    [Obsolete("MTable is now called TableMsg")]
    public class MTable : TableMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTable() { }
        public MTable(MHeader header, Geometry.MPose pose, Geometry.MPoint[] convex_hull) { }
    }

    [Obsolete("MTableArray is now called TableArrayMsg")]
    public class MTableArray : TableArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTableArray() { }
        public MTableArray(MHeader header, MTable[] tables) { }
    }

    [Obsolete("MGetObjectInformationRequest is now called GetObjectInformationRequestMsg")]
    public class MGetObjectInformationRequest : GetObjectInformationRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetObjectInformationRequest() { }
        public MGetObjectInformationRequest(MObjectType type) { }
    }

    [Obsolete("MGetObjectInformationResponse is now called GetObjectInformationResponseMsg")]
    public class MGetObjectInformationResponse : GetObjectInformationResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetObjectInformationResponse() { }
        public MGetObjectInformationResponse(MObjectInformation information) { }
    }
}

namespace RosMessageTypes.Octomap
{
    [Obsolete("MOctomap is now called OctomapMsg")]
    public class MOctomap : OctomapMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MOctomap() { }
        public MOctomap(MHeader header, bool binary, string id, double resolution, sbyte[] data) { }
    }

    [Obsolete("MOctomapWithPose is now called OctomapWithPoseMsg")]
    public class MOctomapWithPose : OctomapWithPoseMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MOctomapWithPose() { }
        public MOctomapWithPose(MHeader header, Geometry.MPose origin, MOctomap octomap) { }
    }

    [Obsolete("MBoundingBoxQueryRequest is now called BoundingBoxQueryRequestMsg")]
    public class MBoundingBoxQueryRequest : BoundingBoxQueryRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MBoundingBoxQueryRequest() { }
        public MBoundingBoxQueryRequest(Geometry.MPoint min, Geometry.MPoint max) { }
    }

    [Obsolete("MBoundingBoxQueryResponse is now called BoundingBoxQueryResponseMsg")]
    public class MBoundingBoxQueryResponse : BoundingBoxQueryResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MBoundingBoxQueryResponse() { }
    }

    [Obsolete("MGetOctomapRequest is now called GetOctomapRequestMsg")]
    public class MGetOctomapRequest : GetOctomapRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetOctomapRequest() { }
    }

    [Obsolete("MGetOctomapResponse is now called GetOctomapResponseMsg")]
    public class MGetOctomapResponse : GetOctomapResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MGetOctomapResponse() { }
        public MGetOctomapResponse(MOctomap map) { }
    }
}

namespace RosMessageTypes.Sensor
{
    [Obsolete("MBatteryState is now called BatteryStateMsg")]
    public class MBatteryState : BatteryStateMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MBatteryState() { }
        public MBatteryState(MHeader header, float voltage, float temperature, float current, float charge, float capacity, float design_capacity, float percentage, byte power_supply_status, byte power_supply_health, byte power_supply_technology, bool present, float[] cell_voltage, float[] cell_temperature, string location, string serial_number) { }
    }

    [Obsolete("MCameraInfo is now called CameraInfoMsg")]
    public class MCameraInfo : CameraInfoMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MCameraInfo() { }
        public MCameraInfo(MHeader header, uint height, uint width, string distortion_model, double[] D, double[] K, double[] R, double[] P, uint binning_x, uint binning_y, MRegionOfInterest roi) { }
    }

    [Obsolete("MChannelFloat32 is now called ChannelFloat32Msg")]
    public class MChannelFloat32 : ChannelFloat32Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MChannelFloat32() { }
        public MChannelFloat32(string name, float[] values) { }
    }

    [Obsolete("MCompressedImage is now called CompressedImageMsg")]
    public class MCompressedImage : CompressedImageMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MCompressedImage() { }
        public MCompressedImage(MHeader header, string format, byte[] data) { }
    }

    [Obsolete("MFluidPressure is now called FluidPressureMsg")]
    public class MFluidPressure : FluidPressureMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MFluidPressure() { }
        public MFluidPressure(MHeader header, double fluid_pressure, double variance) { }
    }

    [Obsolete("MIlluminance is now called IlluminanceMsg")]
    public class MIlluminance : IlluminanceMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MIlluminance() { }
        public MIlluminance(MHeader header, double illuminance, double variance) { }
    }

    [Obsolete("MImage is now called ImageMsg")]
    public class MImage : ImageMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MImage() { }
        public MImage(MHeader header, uint height, uint width, string encoding, byte is_bigendian, uint step, byte[] data) { }
    }

    [Obsolete("MImu is now called ImuMsg")]
    public class MImu : ImuMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MImu() { }
        public MImu(MHeader header, Geometry.MQuaternion orientation, double[] orientation_covariance, Geometry.MVector3 angular_velocity, double[] angular_velocity_covariance, Geometry.MVector3 linear_acceleration, double[] linear_acceleration_covariance) { }
    }

    [Obsolete("MJointState is now called JointStateMsg")]
    public class MJointState : JointStateMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MJointState() { }
        public MJointState(MHeader header, string[] name, double[] position, double[] velocity, double[] effort) { }
    }

    [Obsolete("MJoy is now called JoyMsg")]
    public class MJoy : JoyMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MJoy() { }
        public MJoy(MHeader header, float[] axes, int[] buttons) { }
    }

    [Obsolete("MJoyFeedback is now called JoyFeedbackMsg")]
    public class MJoyFeedback : JoyFeedbackMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MJoyFeedback() { }
        public MJoyFeedback(byte type, byte id, float intensity) { }
    }

    [Obsolete("MJoyFeedbackArray is now called JoyFeedbackArrayMsg")]
    public class MJoyFeedbackArray : JoyFeedbackArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MJoyFeedbackArray() { }
        public MJoyFeedbackArray(MJoyFeedback[] array) { }
    }

    [Obsolete("MLaserEcho is now called LaserEchoMsg")]
    public class MLaserEcho : LaserEchoMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MLaserEcho() { }
        public MLaserEcho(float[] echoes) { }
    }

    [Obsolete("MLaserScan is now called LaserScanMsg")]
    public class MLaserScan : LaserScanMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MLaserScan() { }
        public MLaserScan(MHeader header, float angle_min, float angle_max, float angle_increment, float time_increment, float scan_time, float range_min, float range_max, float[] ranges, float[] intensities) { }
    }

    [Obsolete("MMagneticField is now called MagneticFieldMsg")]
    public class MMagneticField : MagneticFieldMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMagneticField() { }
        public MMagneticField(MHeader header, Geometry.MVector3 magnetic_field, double[] magnetic_field_covariance) { }
    }

    [Obsolete("MMultiDOFJointState is now called MultiDOFJointStateMsg")]
    public class MMultiDOFJointState : MultiDOFJointStateMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMultiDOFJointState() { }
        public MMultiDOFJointState(MHeader header, string[] joint_names, Geometry.MTransform[] transforms, Geometry.MTwist[] twist, Geometry.MWrench[] wrench) { }
    }

    [Obsolete("MMultiEchoLaserScan is now called MultiEchoLaserScanMsg")]
    public class MMultiEchoLaserScan : MultiEchoLaserScanMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMultiEchoLaserScan() { }
        public MMultiEchoLaserScan(MHeader header, float angle_min, float angle_max, float angle_increment, float time_increment, float scan_time, float range_min, float range_max, MLaserEcho[] ranges, MLaserEcho[] intensities) { }
    }

    [Obsolete("MNavSatFix is now called NavSatFixMsg")]
    public class MNavSatFix : NavSatFixMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MNavSatFix() { }
        public MNavSatFix(MHeader header, MNavSatStatus status, double latitude, double longitude, double altitude, double[] position_covariance, byte position_covariance_type) { }
    }

    [Obsolete("MNavSatStatus is now called NavSatStatusMsg")]
    public class MNavSatStatus : NavSatStatusMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MNavSatStatus() { }
        public MNavSatStatus(sbyte status, ushort service) { }
    }

    [Obsolete("MPointCloud is now called PointCloudMsg")]
    public class MPointCloud : PointCloudMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPointCloud() { }
        public MPointCloud(MHeader header, Geometry.MPoint32[] points, MChannelFloat32[] channels) { }
    }

    [Obsolete("MPointCloud2 is now called PointCloud2Msg")]
    public class MPointCloud2 : PointCloud2Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPointCloud2() { }
        public MPointCloud2(MHeader header, uint height, uint width, MPointField[] fields, bool is_bigendian, uint point_step, uint row_step, byte[] data, bool is_dense) { }
    }

    [Obsolete("MPointField is now called PointFieldMsg")]
    public class MPointField : PointFieldMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPointField() { }
        public MPointField(string name, uint offset, byte datatype, uint count) { }
    }

    [Obsolete("MRange is now called RangeMsg")]
    public class MRange : RangeMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MRange() { }
        public MRange(MHeader header, byte radiation_type, float field_of_view, float min_range, float max_range, float range) { }
    }

    [Obsolete("MRegionOfInterest is now called RegionOfInterestMsg")]
    public class MRegionOfInterest : RegionOfInterestMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MRegionOfInterest() { }
        public MRegionOfInterest(uint x_offset, uint y_offset, uint height, uint width, bool do_rectify) { }
    }

    [Obsolete("MRelativeHumidity is now called RelativeHumidityMsg")]
    public class MRelativeHumidity : RelativeHumidityMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MRelativeHumidity() { }
        public MRelativeHumidity(MHeader header, double relative_humidity, double variance) { }
    }

    [Obsolete("MTemperature is now called TemperatureMsg")]
    public class MTemperature : TemperatureMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTemperature() { }
        public MTemperature(MHeader header, double temperature, double variance) { }
    }

    [Obsolete("MTimeReference is now called TimeReferenceMsg")]
    public class MTimeReference : TimeReferenceMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MTimeReference() { }
        public MTimeReference(MHeader header, MTime time_ref, string source) { }
    }

    [Obsolete("MSetCameraInfoRequest is now called SetCameraInfoRequestMsg")]
    public class MSetCameraInfoRequest : SetCameraInfoRequest
    {
        public const string RosMessageName = k_RosMessageName;
        public MSetCameraInfoRequest() { }
        public MSetCameraInfoRequest(MCameraInfo camera_info) { }
    }

    [Obsolete("MSetCameraInfoResponse is now called SetCameraInfoResponseMsg")]
    public class MSetCameraInfoResponse : SetCameraInfoResponse
    {
        public const string RosMessageName = k_RosMessageName;
        public MSetCameraInfoResponse() { }
        public MSetCameraInfoResponse(bool success, string status_message) { }
    }
}

namespace RosMessageTypes.Shape
{
    [Obsolete("MMesh is now called MeshMsg")]
    public class MMesh : MeshMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMesh() { }
        public MMesh(MMeshTriangle[] triangles, Geometry.MPoint[] vertices) { }
    }

    [Obsolete("MMeshTriangle is now called MeshTriangleMsg")]
    public class MMeshTriangle : MeshTriangleMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMeshTriangle() { }
        public MMeshTriangle(uint[] vertex_indices) { }
    }

    [Obsolete("MPlane is now called PlaneMsg")]
    public class MPlane : PlaneMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MPlane() { }
        public MPlane(double[] coef) { }
    }

    [Obsolete("MSolidPrimitive is now called SolidPrimitiveMsg")]
    public class MSolidPrimitive : SolidPrimitiveMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MSolidPrimitive() { }
        public MSolidPrimitive(byte type, double[] dimensions) { }
    }
}

namespace RosMessageTypes.Std
{
    [Obsolete("MBool is now called BoolMsg")]
    public class MBool : BoolMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MBool() { }
        public MBool(bool data) { }
    }

    [Obsolete("MByte is now called ByteMsg")]
    public class MByte : ByteMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MByte() { }
        public MByte(sbyte data) { }
    }

    [Obsolete("MByteMultiArray is now called ByteMultiArrayMsg")]
    public class MByteMultiArray : ByteMultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MByteMultiArray() { }
        public MByteMultiArray(MMultiArrayLayout layout, sbyte[] data) { }
    }

    [Obsolete("MChar is now called CharMsg")]
    public class MChar : CharMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MChar() { }
        public MChar(byte data) { }
    }

    [Obsolete("MColorRGBA is now called ColorRGBAMsg")]
    public class MColorRGBA : ColorRGBAMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MColorRGBA() { }
        public MColorRGBA(float r, float g, float b, float a) { }
    }

    [Obsolete("MDuration is now called DurationMsg")]
    public class MDuration : DurationMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public int secs;
        public int nsecs;
        public MDuration() { }
        public MDuration(int secs, int nsecs) { }
    }

    [Obsolete("MEmpty is now called EmptyMsg")]
    public class MEmpty : EmptyMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MEmpty() { }
    }

    [Obsolete("MFloat32 is now called Float32Msg")]
    public class MFloat32 : Float32Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MFloat32() { }
        public MFloat32(float data) { }
    }

    [Obsolete("MFloat32MultiArray is now called Float32MultiArrayMsg")]
    public class MFloat32MultiArray : Float32MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MFloat32MultiArray() { }
        public MFloat32MultiArray(MMultiArrayLayout layout, float[] data) { }
    }

    [Obsolete("MFloat64 is now called Float64Msg")]
    public class MFloat64 : Float64Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MFloat64() { }
        public MFloat64(double data) { }
    }

    [Obsolete("MFloat64MultiArray is now called Float64MultiArrayMsg")]
    public class MFloat64MultiArray : Float64MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MFloat64MultiArray() { }
        public MFloat64MultiArray(MMultiArrayLayout layout, double[] data) { }
    }

    [Obsolete("MHeader is now called HeaderMsg")]
    public class MHeader : HeaderMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MHeader() { }
        public MHeader(uint seq, MTime stamp, string frame_id) { }
    }

    [Obsolete("MInt16 is now called Int16Msg")]
    public class MInt16 : Int16Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInt16() { }
        public MInt16(short data) { }
    }

    [Obsolete("MInt16MultiArray is now called Int16MultiArrayMsg")]
    public class MInt16MultiArray : Int16MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInt16MultiArray() { }
        public MInt16MultiArray(MMultiArrayLayout layout, short[] data) { }
    }

    [Obsolete("MInt32 is now called Int32Msg")]
    public class MInt32 : Int32Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInt32() { }
        public MInt32(int data) { }
    }

    [Obsolete("MInt32MultiArray is now called Int32MultiArrayMsg")]
    public class MInt32MultiArray : Int32MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInt32MultiArray() { }
        public MInt32MultiArray(MMultiArrayLayout layout, int[] data) { }
    }

    [Obsolete("MInt64 is now called Int64Msg")]
    public class MInt64 : Int64Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInt64() { }
        public MInt64(long data) { }
    }

    [Obsolete("MInt64MultiArray is now called Int64MultiArrayMsg")]
    public class MInt64MultiArray : Int64MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInt64MultiArray() { }
        public MInt64MultiArray(MMultiArrayLayout layout, long[] data) { }
    }

    [Obsolete("MInt8 is now called Int8Msg")]
    public class MInt8 : Int8Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInt8() { }
        public MInt8(sbyte data) { }
    }

    [Obsolete("MInt8MultiArray is now called Int8MultiArrayMsg")]
    public class MInt8MultiArray : Int8MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInt8MultiArray() { }
        public MInt8MultiArray(MMultiArrayLayout layout, sbyte[] data) { }
    }

    [Obsolete("MMultiArrayDimension is now called MultiArrayDimensionMsg")]
    public class MMultiArrayDimension : MultiArrayDimensionMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMultiArrayDimension() { }
        public MMultiArrayDimension(string label, uint size, uint stride) { }
    }

    [Obsolete("MMultiArrayLayout is now called MultiArrayLayoutMsg")]
    public class MMultiArrayLayout : MultiArrayLayoutMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMultiArrayLayout() { }
        public MMultiArrayLayout(MMultiArrayDimension[] dim, uint data_offset) { }
    }

    [Obsolete("MString is now called StringMsg")]
    public class MString : StringMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MString() { }
        public MString(string data) { }
    }

    [Obsolete("MTime is now called TimeMsg")]
    public class MTime : TimeMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public uint secs;
        public uint nsecs;
        public MTime() { }
        public MTime(uint secs, uint nsecs) { }
    }

    [Obsolete("MUInt16 is now called UInt16Msg")]
    public class MUInt16 : UInt16Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MUInt16() { }
        public MUInt16(ushort data) { }
    }

    [Obsolete("MUInt16MultiArray is now called UInt16MultiArrayMsg")]
    public class MUInt16MultiArray : UInt16MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MUInt16MultiArray() { }
        public MUInt16MultiArray(MMultiArrayLayout layout, ushort[] data) { }
    }

    [Obsolete("MUInt32 is now called UInt32Msg")]
    public class MUInt32 : UInt32Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MUInt32() { }
        public MUInt32(uint data) { }
    }

    [Obsolete("MUInt32MultiArray is now called UInt32MultiArrayMsg")]
    public class MUInt32MultiArray : UInt32MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MUInt32MultiArray() { }
        public MUInt32MultiArray(MMultiArrayLayout layout, uint[] data) { }
    }

    [Obsolete("MUInt64 is now called UInt64Msg")]
    public class MUInt64 : UInt64Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MUInt64() { }
        public MUInt64(ulong data) { }
    }

    [Obsolete("MUInt64MultiArray is now called UInt64MultiArrayMsg")]
    public class MUInt64MultiArray : UInt64MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MUInt64MultiArray() { }
        public MUInt64MultiArray(MMultiArrayLayout layout, ulong[] data) { }
    }

    [Obsolete("MUInt8 is now called UInt8Msg")]
    public class MUInt8 : UInt8Msg
    {
        public const string RosMessageName = k_RosMessageName;
        public MUInt8() { }
        public MUInt8(byte data) { }
    }

    [Obsolete("MUInt8MultiArray is now called UInt8MultiArrayMsg")]
    public class MUInt8MultiArray : UInt8MultiArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MUInt8MultiArray() { }
        public MUInt8MultiArray(MMultiArrayLayout layout, byte[] data) { }
    }
}

namespace RosMessageTypes.Stereo
{
    [Obsolete("MDisparityImage is now called DisparityImageMsg")]
    public class MDisparityImage : DisparityImageMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MDisparityImage() { }
        public MDisparityImage(MHeader header, Sensor.MImage image, float f, float T, Sensor.MRegionOfInterest valid_window, float min_disparity, float max_disparity, float delta_d) { }
    }
}

namespace RosMessageTypes.Trajectory
{
    [Obsolete("MJointTrajectory is now called JointTrajectoryMsg")]
    public class MJointTrajectory : JointTrajectoryMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MJointTrajectory() { }
        public MJointTrajectory(MHeader header, string[] joint_names, MJointTrajectoryPoint[] points) { }
    }

    [Obsolete("MJointTrajectoryPoint is now called JointTrajectoryPointMsg")]
    public class MJointTrajectoryPoint : JointTrajectoryPointMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MJointTrajectoryPoint() { }
        public MJointTrajectoryPoint(double[] positions, double[] velocities, double[] accelerations, double[] effort, MDuration time_from_start) { }
    }

    [Obsolete("MMultiDOFJointTrajectory is now called MultiDOFJointTrajectoryMsg")]
    public class MMultiDOFJointTrajectory : MultiDOFJointTrajectoryMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMultiDOFJointTrajectory() { }
        public MMultiDOFJointTrajectory(MHeader header, string[] joint_names, MMultiDOFJointTrajectoryPoint[] points) { }
    }

    [Obsolete("MMultiDOFJointTrajectoryPoint is now called MultiDOFJointTrajectoryPointMsg")]
    public class MMultiDOFJointTrajectoryPoint : MultiDOFJointTrajectoryPointMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMultiDOFJointTrajectoryPoint() { }
        public MMultiDOFJointTrajectoryPoint(Geometry.MTransform[] transforms, Geometry.MTwist[] velocities, Geometry.MTwist[] accelerations, MDuration time_from_start) { }
    }
}

namespace RosMessageTypes.Visualization
{
    [Obsolete("MImageMarker is now called ImageMarkerMsg")]
    public class MImageMarker : ImageMarkerMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MImageMarker() { }
        public MImageMarker(MHeader header, string ns, int id, int type, int action, Geometry.MPoint position, float scale, Std.MColorRGBA outline_color, byte filled, Std.MColorRGBA fill_color, MDuration lifetime, Geometry.MPoint[] points, Std.MColorRGBA[] outline_colors) { }
    }

    [Obsolete("MInteractiveMarker is now called InteractiveMarkerMsg")]
    public class MInteractiveMarker : InteractiveMarkerMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInteractiveMarker() { }
        public MInteractiveMarker(MHeader header, Geometry.MPose pose, string name, string description, float scale, MMenuEntry[] menu_entries, MInteractiveMarkerControl[] controls) { }
    }

    [Obsolete("MInteractiveMarkerControl is now called InteractiveMarkerControlMsg")]
    public class MInteractiveMarkerControl : InteractiveMarkerControlMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerControl() { }
        public MInteractiveMarkerControl(string name, Geometry.MQuaternion orientation, byte orientation_mode, byte interaction_mode, bool always_visible, MMarker[] markers, bool independent_marker_orientation, string description) { }
    }

    [Obsolete("MInteractiveMarkerFeedback is now called InteractiveMarkerFeedbackMsg")]
    public class MInteractiveMarkerFeedback : InteractiveMarkerFeedbackMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerFeedback() { }
        public MInteractiveMarkerFeedback(MHeader header, string client_id, string marker_name, string control_name, byte event_type, Geometry.MPose pose, uint menu_entry_id, Geometry.MPoint mouse_point, bool mouse_point_valid) { }
    }

    [Obsolete("MInteractiveMarkerInit is now called InteractiveMarkerInitMsg")]
    public class MInteractiveMarkerInit : InteractiveMarkerInitMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerInit() { }
        public MInteractiveMarkerInit(string server_id, ulong seq_num, MInteractiveMarker[] markers) { }
    }

    [Obsolete("MInteractiveMarkerPose is now called InteractiveMarkerPoseMsg")]
    public class MInteractiveMarkerPose : InteractiveMarkerPoseMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerPose() { }
        public MInteractiveMarkerPose(MHeader header, Geometry.MPose pose, string name) { }
    }

    [Obsolete("MInteractiveMarkerUpdate is now called InteractiveMarkerUpdateMsg")]
    public class MInteractiveMarkerUpdate : InteractiveMarkerUpdateMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerUpdate() { }
        public MInteractiveMarkerUpdate(string server_id, ulong seq_num, byte type, MInteractiveMarker[] markers, MInteractiveMarkerPose[] poses, string[] erases) { }
    }

    [Obsolete("MMarker is now called MarkerMsg")]
    public class MMarker : MarkerMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMarker() { }
        public MMarker(MHeader header, string ns, int id, int type, int action, Geometry.MPose pose, Geometry.MVector3 scale, Std.MColorRGBA color, MDuration lifetime, bool frame_locked, Geometry.MPoint[] points, Std.MColorRGBA[] colors, string text, string mesh_resource, bool mesh_use_embedded_materials) { }
    }

    [Obsolete("MMarkerArray is now called MarkerArrayMsg")]
    public class MMarkerArray : MarkerArrayMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMarkerArray() { }
        public MMarkerArray(MMarker[] markers) { }
    }

    [Obsolete("MMenuEntry is now called MenuEntryMsg")]
    public class MMenuEntry : MenuEntryMsg
    {
        public const string RosMessageName = k_RosMessageName;
        public MMenuEntry() { }
        public MMenuEntry(uint id, uint parent_id, string title, string command, byte command_type) { }
    }
}
