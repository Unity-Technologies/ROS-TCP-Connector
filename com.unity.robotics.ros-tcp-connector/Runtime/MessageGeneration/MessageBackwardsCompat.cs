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
        public new const string RosMessageName = k_RosMessageName;
        public MGoalID() => throw new NotImplementedException("MGoalID is now called GoalIDMsg");
        public MGoalID(MTime stamp, string id) => throw new NotImplementedException("MGoalID is now called GoalIDMsg");
    }

    [Obsolete("MGoalStatus is now called GoalStatusMsg")]
    public class MGoalStatus : GoalStatusMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGoalStatus() => throw new NotImplementedException("MGoalStatus is now called GoalStatusMsg");
        public MGoalStatus(MGoalID goal_id, byte status, string text) => throw new NotImplementedException("MGoalStatus is now called GoalStatusMsg");
    }

    [Obsolete("MGoalStatusArray is now called GoalStatusArrayMsg")]
    public class MGoalStatusArray : GoalStatusArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGoalStatusArray() => throw new NotImplementedException("MGoalStatusArray is now called GoalStatusArrayMsg");
        public MGoalStatusArray(MHeader header, MGoalStatus[] status_list) => throw new NotImplementedException("MGoalStatusArray is now called GoalStatusArrayMsg");
    }
}

namespace RosMessageTypes.Diagnostic
{
    [Obsolete("MDiagnosticArray is now called DiagnosticArrayMsg")]
    public class MDiagnosticArray : DiagnosticArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MDiagnosticArray() => throw new NotImplementedException("MDiagnosticArray is now called DiagnosticArrayMsg");
        public MDiagnosticArray(MHeader header, MDiagnosticStatus[] status) => throw new NotImplementedException("MDiagnosticArray is now called DiagnosticArrayMsg");
    }

    [Obsolete("MDiagnosticStatus is now called DiagnosticStatusMsg")]
    public class MDiagnosticStatus : DiagnosticStatusMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MDiagnosticStatus() => throw new NotImplementedException("MDiagnosticStatus is now called DiagnosticStatusMsg");
        public MDiagnosticStatus(sbyte level, string name, string message, string hardware_id, MKeyValue[] values) => throw new NotImplementedException("MDiagnosticStatus is now called DiagnosticStatusMsg");
    }

    [Obsolete("MKeyValue is now called KeyValueMsg")]
    public class MKeyValue : KeyValueMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MKeyValue() => throw new NotImplementedException("MKeyValue is now called KeyValueMsg");
        public MKeyValue(string key, string value) => throw new NotImplementedException("MKeyValue is now called KeyValueMsg");
    }

    [Obsolete("MAddDiagnosticsRequest is now called AddDiagnosticsRequest")]
    public class MAddDiagnosticsRequest : AddDiagnosticsRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MAddDiagnosticsRequest() => throw new NotImplementedException("MAddDiagnosticsRequest is now called AddDiagnosticsRequest");
        public MAddDiagnosticsRequest(string load_namespace) => throw new NotImplementedException("MAddDiagnosticsRequest is now called AddDiagnosticsRequest");
    }

    [Obsolete("MAddDiagnosticsResponse is now called AddDiagnosticsResponse")]
    public class MAddDiagnosticsResponse : AddDiagnosticsResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MAddDiagnosticsResponse() => throw new NotImplementedException("MAddDiagnosticsResponse is now called AddDiagnosticsResponse");
        public MAddDiagnosticsResponse(bool success, string message) => throw new NotImplementedException("MAddDiagnosticsResponse is now called AddDiagnosticsResponse");
    }

    [Obsolete("MSelfTestRequest is now called SelfTestRequest")]
    public class MSelfTestRequest : SelfTestRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MSelfTestRequest() => throw new NotImplementedException("MSelfTestRequest is now called SelfTestRequest");
    }

    [Obsolete("MSelfTestResponse is now called SelfTestResponse")]
    public class MSelfTestResponse : SelfTestResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MSelfTestResponse() => throw new NotImplementedException("MSelfTestResponse is now called SelfTestResponse");
        public MSelfTestResponse(string id, sbyte passed, MDiagnosticStatus[] status) => throw new NotImplementedException("MSelfTestResponse is now called SelfTestResponse");
    }
}

namespace RosMessageTypes.Geometry
{
    [Obsolete("MAccel is now called AccelMsg")]
    public class MAccel : AccelMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MAccel() => throw new NotImplementedException("MAccel is now called AccelMsg");
        public MAccel(MVector3 linear, MVector3 angular) => throw new NotImplementedException("MAccel is now called AccelMsg");
    }

    [Obsolete("MAccelStamped is now called AccelStampedMsg")]
    public class MAccelStamped : AccelStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MAccelStamped() => throw new NotImplementedException("MAccelStamped is now called AccelStampedMsg");
        public MAccelStamped(MHeader header, MAccel accel) => throw new NotImplementedException("MAccelStamped is now called AccelStampedMsg");
    }

    [Obsolete("MAccelWithCovariance is now called AccelWithCovarianceMsg")]
    public class MAccelWithCovariance : AccelWithCovarianceMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MAccelWithCovariance() => throw new NotImplementedException("MAccelWithCovariance is now called AccelWithCovarianceMsg");
        public MAccelWithCovariance(MAccel accel, double[] covariance) => throw new NotImplementedException("MAccelWithCovariance is now called AccelWithCovarianceMsg");
    }

    [Obsolete("MAccelWithCovarianceStamped is now called AccelWithCovarianceStampedMsg")]
    public class MAccelWithCovarianceStamped : AccelWithCovarianceStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MAccelWithCovarianceStamped() => throw new NotImplementedException("MAccelWithCovarianceStamped is now called AccelWithCovarianceStampedMsg");
        public MAccelWithCovarianceStamped(MHeader header, MAccelWithCovariance accel) => throw new NotImplementedException("MAccelWithCovarianceStamped is now called AccelWithCovarianceStampedMsg");
    }

    [Obsolete("MInertia is now called InertiaMsg")]
    public class MInertia : InertiaMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInertia() => throw new NotImplementedException("MInertia is now called InertiaMsg");
        public MInertia(double m, MVector3 com, double ixx, double ixy, double ixz, double iyy, double iyz, double izz) => throw new NotImplementedException("MInertia is now called InertiaMsg");
    }

    [Obsolete("MInertiaStamped is now called InertiaStampedMsg")]
    public class MInertiaStamped : InertiaStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInertiaStamped() => throw new NotImplementedException("MInertiaStamped is now called InertiaStampedMsg");
        public MInertiaStamped(MHeader header, MInertia inertia) => throw new NotImplementedException("MInertiaStamped is now called InertiaStampedMsg");
    }

    [Obsolete("MPoint is now called PointMsg")]
    public class MPoint : PointMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPoint() => throw new NotImplementedException("MPoint is now called PointMsg");
        public MPoint(double x, double y, double z) => throw new NotImplementedException("MPoint is now called PointMsg");
    }

    [Obsolete("MPoint32 is now called Point32Msg")]
    public class MPoint32 : Point32Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPoint32() => throw new NotImplementedException("MPoint32 is now called Point32Msg");
        public MPoint32(float x, float y, float z) => throw new NotImplementedException("MPoint32 is now called Point32Msg");
    }

    [Obsolete("MPointStamped is now called PointStampedMsg")]
    public class MPointStamped : PointStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPointStamped() => throw new NotImplementedException("MPointStamped is now called PointStampedMsg");
        public MPointStamped(MHeader header, MPoint point) => throw new NotImplementedException("MPointStamped is now called PointStampedMsg");
    }

    [Obsolete("MPolygon is now called PolygonMsg")]
    public class MPolygon : PolygonMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPolygon() => throw new NotImplementedException("MPolygon is now called PolygonMsg");
        public MPolygon(MPoint32[] points) => throw new NotImplementedException("MPolygon is now called PolygonMsg");
    }

    [Obsolete("MPolygonStamped is now called PolygonStampedMsg")]
    public class MPolygonStamped : PolygonStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPolygonStamped() => throw new NotImplementedException("MPolygonStamped is now called PolygonStampedMsg");
        public MPolygonStamped(MHeader header, MPolygon polygon) => throw new NotImplementedException("MPolygonStamped is now called PolygonStampedMsg");
    }

    [Obsolete("MPose is now called PoseMsg")]
    public class MPose : PoseMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPose() => throw new NotImplementedException("MPose is now called PoseMsg");
        public MPose(MPoint position, MQuaternion orientation) => throw new NotImplementedException("MPose is now called PoseMsg");
    }

    [Obsolete("MPose2D is now called Pose2DMsg")]
    public class MPose2D : Pose2DMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPose2D() => throw new NotImplementedException("MPose2D is now called Pose2DMsg");
        public MPose2D(double x, double y, double theta) => throw new NotImplementedException("MPose2D is now called Pose2DMsg");
    }

    [Obsolete("MPoseArray is now called PoseArrayMsg")]
    public class MPoseArray : PoseArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPoseArray() => throw new NotImplementedException("MPoseArray is now called PoseArrayMsg");
        public MPoseArray(MHeader header, MPose[] poses) => throw new NotImplementedException("MPoseArray is now called PoseArrayMsg");
    }

    [Obsolete("MPoseStamped is now called PoseStampedMsg")]
    public class MPoseStamped : PoseStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPoseStamped() => throw new NotImplementedException("MPoseStamped is now called PoseStampedMsg");
        public MPoseStamped(MHeader header, MPose pose) => throw new NotImplementedException("MPoseStamped is now called PoseStampedMsg");
    }

    [Obsolete("MPoseWithCovariance is now called PoseWithCovarianceMsg")]
    public class MPoseWithCovariance : PoseWithCovarianceMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPoseWithCovariance() => throw new NotImplementedException("MPoseWithCovariance is now called PoseWithCovarianceMsg");
        public MPoseWithCovariance(MPose pose, double[] covariance) => throw new NotImplementedException("MPoseWithCovariance is now called PoseWithCovarianceMsg");
    }

    [Obsolete("MPoseWithCovarianceStamped is now called PoseWithCovarianceStampedMsg")]
    public class MPoseWithCovarianceStamped : PoseWithCovarianceStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPoseWithCovarianceStamped() => throw new NotImplementedException("MPoseWithCovarianceStamped is now called PoseWithCovarianceStampedMsg");
        public MPoseWithCovarianceStamped(MHeader header, MPoseWithCovariance pose) => throw new NotImplementedException("MPoseWithCovarianceStamped is now called PoseWithCovarianceStampedMsg");
    }

    [Obsolete("MQuaternion is now called QuaternionMsg")]
    public class MQuaternion : QuaternionMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MQuaternion() => throw new NotImplementedException("MQuaternion is now called QuaternionMsg");
        public MQuaternion(double x, double y, double z, double w) => throw new NotImplementedException("MQuaternion is now called QuaternionMsg");
    }

    [Obsolete("MQuaternionStamped is now called QuaternionStampedMsg")]
    public class MQuaternionStamped : QuaternionStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MQuaternionStamped() => throw new NotImplementedException("MQuaternionStamped is now called QuaternionStampedMsg");
        public MQuaternionStamped(MHeader header, MQuaternion quaternion) => throw new NotImplementedException("MQuaternionStamped is now called QuaternionStampedMsg");
    }

    [Obsolete("MTransform is now called TransformMsg")]
    public class MTransform : TransformMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTransform() => throw new NotImplementedException("MTransform is now called TransformMsg");
        public MTransform(MVector3 translation, MQuaternion rotation) => throw new NotImplementedException("MTransform is now called TransformMsg");
    }

    [Obsolete("MTransformStamped is now called TransformStampedMsg")]
    public class MTransformStamped : TransformStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTransformStamped() => throw new NotImplementedException("MTransformStamped is now called TransformStampedMsg");
        public MTransformStamped(MHeader header, string child_frame_id, MTransform transform) => throw new NotImplementedException("MTransformStamped is now called TransformStampedMsg");
    }

    [Obsolete("MTwist is now called TwistMsg")]
    public class MTwist : TwistMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTwist() => throw new NotImplementedException("MTwist is now called TwistMsg");
        public MTwist(MVector3 linear, MVector3 angular) => throw new NotImplementedException("MTwist is now called TwistMsg");
    }

    [Obsolete("MTwistStamped is now called TwistStampedMsg")]
    public class MTwistStamped : TwistStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTwistStamped() => throw new NotImplementedException("MTwistStamped is now called TwistStampedMsg");
        public MTwistStamped(MHeader header, MTwist twist) => throw new NotImplementedException("MTwistStamped is now called TwistStampedMsg");
    }

    [Obsolete("MTwistWithCovariance is now called TwistWithCovarianceMsg")]
    public class MTwistWithCovariance : TwistWithCovarianceMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTwistWithCovariance() => throw new NotImplementedException("MTwistWithCovariance is now called TwistWithCovarianceMsg");
        public MTwistWithCovariance(MTwist twist, double[] covariance) => throw new NotImplementedException("MTwistWithCovariance is now called TwistWithCovarianceMsg");
    }

    [Obsolete("MTwistWithCovarianceStamped is now called TwistWithCovarianceStampedMsg")]
    public class MTwistWithCovarianceStamped : TwistWithCovarianceStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTwistWithCovarianceStamped() => throw new NotImplementedException("MTwistWithCovarianceStamped is now called TwistWithCovarianceStampedMsg");
        public MTwistWithCovarianceStamped(MHeader header, MTwistWithCovariance twist) => throw new NotImplementedException("MTwistWithCovarianceStamped is now called TwistWithCovarianceStampedMsg");
    }

    [Obsolete("MVector3 is now called Vector3Msg")]
    public class MVector3 : Vector3Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MVector3() => throw new NotImplementedException("MVector3 is now called Vector3Msg");
        public MVector3(double x, double y, double z) => throw new NotImplementedException("MVector3 is now called Vector3Msg");
    }

    [Obsolete("MVector3Stamped is now called Vector3StampedMsg")]
    public class MVector3Stamped : Vector3StampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MVector3Stamped() => throw new NotImplementedException("MVector3Stamped is now called Vector3StampedMsg");
        public MVector3Stamped(MHeader header, MVector3 vector) => throw new NotImplementedException("MVector3Stamped is now called Vector3StampedMsg");
    }

    [Obsolete("MWrench is now called WrenchMsg")]
    public class MWrench : WrenchMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MWrench() => throw new NotImplementedException("MWrench is now called WrenchMsg");
        public MWrench(MVector3 force, MVector3 torque) => throw new NotImplementedException("MWrench is now called WrenchMsg");
    }

    [Obsolete("MWrenchStamped is now called WrenchStampedMsg")]
    public class MWrenchStamped : WrenchStampedMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MWrenchStamped() => throw new NotImplementedException("MWrenchStamped is now called WrenchStampedMsg");
        public MWrenchStamped(MHeader header, MWrench wrench) => throw new NotImplementedException("MWrenchStamped is now called WrenchStampedMsg");
    }
}

namespace RosMessageTypes.Nav
{
    [Obsolete("MGetMapFeedback is now called GetMapFeedback")]
    public class MGetMapFeedback : GetMapFeedback
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetMapFeedback() => throw new NotImplementedException("MGetMapFeedback is now called GetMapFeedback");
    }

    [Obsolete("MGetMapGoal is now called GetMapGoal")]
    public class MGetMapGoal : GetMapGoal
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetMapGoal() => throw new NotImplementedException("MGetMapGoal is now called GetMapGoal");
    }

    [Obsolete("MGetMapResult is now called GetMapResult")]
    public class MGetMapResult : GetMapResult
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetMapResult() => throw new NotImplementedException("MGetMapResult is now called GetMapResult");
        public MGetMapResult(MOccupancyGrid map) => throw new NotImplementedException("MGetMapResult is now called GetMapResult");
    }

    [Obsolete("MGridCells is now called GridCellsMsg")]
    public class MGridCells : GridCellsMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGridCells() => throw new NotImplementedException("MGridCells is now called GridCellsMsg");
        public MGridCells(MHeader header, float cell_width, float cell_height, Geometry.MPoint[] cells) => throw new NotImplementedException("MGridCells is now called GridCellsMsg");
    }

    [Obsolete("MMapMetaData is now called MapMetaDataMsg")]
    public class MMapMetaData : MapMetaDataMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMapMetaData() => throw new NotImplementedException("MMapMetaData is now called MapMetaDataMsg");
        public MMapMetaData(MTime map_load_time, float resolution, uint width, uint height, Geometry.MPose origin) => throw new NotImplementedException("MMapMetaData is now called MapMetaDataMsg");
    }

    [Obsolete("MOccupancyGrid is now called OccupancyGridMsg")]
    public class MOccupancyGrid : OccupancyGridMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MOccupancyGrid() => throw new NotImplementedException("MOccupancyGrid is now called OccupancyGridMsg");
        public MOccupancyGrid(MHeader header, MMapMetaData info, sbyte[] data) => throw new NotImplementedException("MOccupancyGrid is now called OccupancyGridMsg");
    }

    [Obsolete("MOdometry is now called OdometryMsg")]
    public class MOdometry : OdometryMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MOdometry() => throw new NotImplementedException("MOdometry is now called OdometryMsg");
        public MOdometry(MHeader header, string child_frame_id, Geometry.MPoseWithCovariance pose, Geometry.MTwistWithCovariance twist) => throw new NotImplementedException("MOdometry is now called OdometryMsg");
    }

    [Obsolete("MPath is now called PathMsg")]
    public class MPath : PathMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPath() => throw new NotImplementedException("MPath is now called PathMsg");
        public MPath(MHeader header, Geometry.MPoseStamped[] poses) => throw new NotImplementedException("MPath is now called PathMsg");
    }

    [Obsolete("MGetMapRequest is now called GetMapRequest")]
    public class MGetMapRequest : GetMapRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetMapRequest() => throw new NotImplementedException("MGetMapRequest is now called GetMapRequest");
    }

    [Obsolete("MGetMapResponse is now called GetMapResponse")]
    public class MGetMapResponse : GetMapResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetMapResponse() => throw new NotImplementedException("MGetMapResponse is now called GetMapResponse");
        public MGetMapResponse(MOccupancyGrid map) => throw new NotImplementedException("MGetMapResponse is now called GetMapResponse");
    }

    [Obsolete("MGetPlanRequest is now called GetPlanRequest")]
    public class MGetPlanRequest : GetPlanRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetPlanRequest() => throw new NotImplementedException("MGetPlanRequest is now called GetPlanRequest");
        public MGetPlanRequest(Geometry.MPoseStamped start, Geometry.MPoseStamped goal, float tolerance) => throw new NotImplementedException("MGetPlanRequest is now called GetPlanRequest");
    }

    [Obsolete("MGetPlanResponse is now called GetPlanResponse")]
    public class MGetPlanResponse : GetPlanResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetPlanResponse() => throw new NotImplementedException("MGetPlanResponse is now called GetPlanResponse");
        public MGetPlanResponse(MPath plan) => throw new NotImplementedException("MGetPlanResponse is now called GetPlanResponse");
    }

    [Obsolete("MLoadMapRequest is now called LoadMapRequest")]
    public class MLoadMapRequest : LoadMapRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MLoadMapRequest() => throw new NotImplementedException("MLoadMapRequest is now called LoadMapRequest");
        public MLoadMapRequest(string map_url) => throw new NotImplementedException("MLoadMapRequest is now called LoadMapRequest");
    }

    [Obsolete("MLoadMapResponse is now called LoadMapResponse")]
    public class MLoadMapResponse : LoadMapResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MLoadMapResponse() => throw new NotImplementedException("MLoadMapResponse is now called LoadMapResponse");
        public MLoadMapResponse(MOccupancyGrid map, byte result) => throw new NotImplementedException("MLoadMapResponse is now called LoadMapResponse");
    }

    [Obsolete("MSetMapRequest is now called SetMapRequest")]
    public class MSetMapRequest : SetMapRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MSetMapRequest() => throw new NotImplementedException("MSetMapRequest is now called SetMapRequest");
        public MSetMapRequest(MOccupancyGrid map, Geometry.MPoseWithCovarianceStamped initial_pose) => throw new NotImplementedException("MSetMapRequest is now called SetMapRequest");
    }

    [Obsolete("MSetMapResponse is now called SetMapResponse")]
    public class MSetMapResponse : SetMapResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MSetMapResponse() => throw new NotImplementedException("MSetMapResponse is now called SetMapResponse");
        public MSetMapResponse(bool success) => throw new NotImplementedException("MSetMapResponse is now called SetMapResponse");
    }
}

namespace RosMessageTypes.ObjectRecognition
{
    [Obsolete("MObjectRecognitionFeedback is now called ObjectRecognitionFeedback")]
    public class MObjectRecognitionFeedback : ObjectRecognitionFeedback
    {
        public new const string RosMessageName = k_RosMessageName;
        public MObjectRecognitionFeedback() => throw new NotImplementedException("MObjectRecognitionFeedback is now called ObjectRecognitionFeedback");
    }

    [Obsolete("MObjectRecognitionGoal is now called ObjectRecognitionGoal")]
    public class MObjectRecognitionGoal : ObjectRecognitionGoal
    {
        public new const string RosMessageName = k_RosMessageName;
        public MObjectRecognitionGoal() => throw new NotImplementedException("MObjectRecognitionGoal is now called ObjectRecognitionGoal");
        public MObjectRecognitionGoal(bool use_roi, float[] filter_limits) => throw new NotImplementedException("MObjectRecognitionGoal is now called ObjectRecognitionGoal");
    }

    [Obsolete("MObjectRecognitionResult is now called ObjectRecognitionResult")]
    public class MObjectRecognitionResult : ObjectRecognitionResult
    {
        public new const string RosMessageName = k_RosMessageName;
        public MObjectRecognitionResult() => throw new NotImplementedException("MObjectRecognitionResult is now called ObjectRecognitionResult");
        public MObjectRecognitionResult(MRecognizedObjectArray recognized_objects) => throw new NotImplementedException("MObjectRecognitionResult is now called ObjectRecognitionResult");
    }

    [Obsolete("MObjectInformation is now called ObjectInformationMsg")]
    public class MObjectInformation : ObjectInformationMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MObjectInformation() => throw new NotImplementedException("MObjectInformation is now called ObjectInformationMsg");
        public MObjectInformation(string name, Shape.MMesh ground_truth_mesh, Sensor.MPointCloud2 ground_truth_point_cloud) => throw new NotImplementedException("MObjectInformation is now called ObjectInformationMsg");
    }

    [Obsolete("MObjectType is now called ObjectTypeMsg")]
    public class MObjectType : ObjectTypeMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MObjectType() => throw new NotImplementedException("MObjectType is now called ObjectTypeMsg");
        public MObjectType(string key, string db) => throw new NotImplementedException("MObjectType is now called ObjectTypeMsg");
    }

    [Obsolete("MRecognizedObject is now called RecognizedObjectMsg")]
    public class MRecognizedObject : RecognizedObjectMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MRecognizedObject() => throw new NotImplementedException("MRecognizedObject is now called RecognizedObjectMsg");
        public MRecognizedObject(MHeader header, MObjectType type, float confidence, Sensor.MPointCloud2[] point_clouds, Shape.MMesh bounding_mesh, Geometry.MPoint[] bounding_contours, Geometry.MPoseWithCovarianceStamped pose) => throw new NotImplementedException("MRecognizedObject is now called RecognizedObjectMsg");
    }

    [Obsolete("MRecognizedObjectArray is now called RecognizedObjectArrayMsg")]
    public class MRecognizedObjectArray : RecognizedObjectArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MRecognizedObjectArray() => throw new NotImplementedException("MRecognizedObjectArray is now called RecognizedObjectArrayMsg");
        public MRecognizedObjectArray(MHeader header, MRecognizedObject[] objects, float[] cooccurrence) => throw new NotImplementedException("MRecognizedObjectArray is now called RecognizedObjectArrayMsg");
    }

    [Obsolete("MTable is now called TableMsg")]
    public class MTable : TableMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTable() => throw new NotImplementedException("MTable is now called TableMsg");
        public MTable(MHeader header, Geometry.MPose pose, Geometry.MPoint[] convex_hull) => throw new NotImplementedException("MTable is now called TableMsg");
    }

    [Obsolete("MTableArray is now called TableArrayMsg")]
    public class MTableArray : TableArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTableArray() => throw new NotImplementedException("MTableArray is now called TableArrayMsg");
        public MTableArray(MHeader header, MTable[] tables) => throw new NotImplementedException("MTableArray is now called TableArrayMsg");
    }

    [Obsolete("MGetObjectInformationRequest is now called GetObjectInformationRequest")]
    public class MGetObjectInformationRequest : GetObjectInformationRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetObjectInformationRequest() => throw new NotImplementedException("MGetObjectInformationRequest is now called GetObjectInformationRequest");
        public MGetObjectInformationRequest(MObjectType type) => throw new NotImplementedException("MGetObjectInformationRequest is now called GetObjectInformationRequest");
    }

    [Obsolete("MGetObjectInformationResponse is now called GetObjectInformationResponse")]
    public class MGetObjectInformationResponse : GetObjectInformationResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetObjectInformationResponse() => throw new NotImplementedException("MGetObjectInformationResponse is now called GetObjectInformationResponse");
        public MGetObjectInformationResponse(MObjectInformation information) => throw new NotImplementedException("MGetObjectInformationResponse is now called GetObjectInformationResponse");
    }
}

namespace RosMessageTypes.Octomap
{
    [Obsolete("MOctomap is now called OctomapMsg")]
    public class MOctomap : OctomapMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MOctomap() => throw new NotImplementedException("MOctomap is now called OctomapMsg");
        public MOctomap(MHeader header, bool binary, string id, double resolution, sbyte[] data) => throw new NotImplementedException("MOctomap is now called OctomapMsg");
    }

    [Obsolete("MOctomapWithPose is now called OctomapWithPoseMsg")]
    public class MOctomapWithPose : OctomapWithPoseMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MOctomapWithPose() => throw new NotImplementedException("MOctomapWithPose is now called OctomapWithPoseMsg");
        public MOctomapWithPose(MHeader header, Geometry.MPose origin, MOctomap octomap) => throw new NotImplementedException("MOctomapWithPose is now called OctomapWithPoseMsg");
    }

    [Obsolete("MBoundingBoxQueryRequest is now called BoundingBoxQueryRequest")]
    public class MBoundingBoxQueryRequest : BoundingBoxQueryRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MBoundingBoxQueryRequest() => throw new NotImplementedException("MBoundingBoxQueryRequest is now called BoundingBoxQueryRequest");
        public MBoundingBoxQueryRequest(Geometry.MPoint min, Geometry.MPoint max) => throw new NotImplementedException("MBoundingBoxQueryRequest is now called BoundingBoxQueryRequest");
    }

    [Obsolete("MBoundingBoxQueryResponse is now called BoundingBoxQueryResponse")]
    public class MBoundingBoxQueryResponse : BoundingBoxQueryResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MBoundingBoxQueryResponse() => throw new NotImplementedException("MBoundingBoxQueryResponse is now called BoundingBoxQueryResponse");
    }

    [Obsolete("MGetOctomapRequest is now called GetOctomapRequest")]
    public class MGetOctomapRequest : GetOctomapRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetOctomapRequest() => throw new NotImplementedException("MGetOctomapRequest is now called GetOctomapRequest");
    }

    [Obsolete("MGetOctomapResponse is now called GetOctomapResponse")]
    public class MGetOctomapResponse : GetOctomapResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MGetOctomapResponse() => throw new NotImplementedException("MGetOctomapResponse is now called GetOctomapResponse");
        public MGetOctomapResponse(MOctomap map) => throw new NotImplementedException("MGetOctomapResponse is now called GetOctomapResponse");
    }
}

namespace RosMessageTypes.Sensor
{
    [Obsolete("MBatteryState is now called BatteryStateMsg")]
    public class MBatteryState : BatteryStateMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MBatteryState() => throw new NotImplementedException("MBatteryState is now called BatteryStateMsg");
        public MBatteryState(MHeader header, float voltage, float temperature, float current, float charge, float capacity, float design_capacity, float percentage, byte power_supply_status, byte power_supply_health, byte power_supply_technology, bool present, float[] cell_voltage, float[] cell_temperature, string location, string serial_number) => throw new NotImplementedException("MBatteryState is now called BatteryStateMsg");
    }

    [Obsolete("MCameraInfo is now called CameraInfoMsg")]
    public class MCameraInfo : CameraInfoMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MCameraInfo() => throw new NotImplementedException("MCameraInfo is now called CameraInfoMsg");
        public MCameraInfo(MHeader header, uint height, uint width, string distortion_model, double[] D, double[] K, double[] R, double[] P, uint binning_x, uint binning_y, MRegionOfInterest roi) => throw new NotImplementedException("MCameraInfo is now called CameraInfoMsg");
    }

    [Obsolete("MChannelFloat32 is now called ChannelFloat32Msg")]
    public class MChannelFloat32 : ChannelFloat32Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MChannelFloat32() => throw new NotImplementedException("MChannelFloat32 is now called ChannelFloat32Msg");
        public MChannelFloat32(string name, float[] values) => throw new NotImplementedException("MChannelFloat32 is now called ChannelFloat32Msg");
    }

    [Obsolete("MCompressedImage is now called CompressedImageMsg")]
    public class MCompressedImage : CompressedImageMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MCompressedImage() => throw new NotImplementedException("MCompressedImage is now called CompressedImageMsg");
        public MCompressedImage(MHeader header, string format, byte[] data) => throw new NotImplementedException("MCompressedImage is now called CompressedImageMsg");
    }

    [Obsolete("MFluidPressure is now called FluidPressureMsg")]
    public class MFluidPressure : FluidPressureMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MFluidPressure() => throw new NotImplementedException("MFluidPressure is now called FluidPressureMsg");
        public MFluidPressure(MHeader header, double fluid_pressure, double variance) => throw new NotImplementedException("MFluidPressure is now called FluidPressureMsg");
    }

    [Obsolete("MIlluminance is now called IlluminanceMsg")]
    public class MIlluminance : IlluminanceMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MIlluminance() => throw new NotImplementedException("MIlluminance is now called IlluminanceMsg");
        public MIlluminance(MHeader header, double illuminance, double variance) => throw new NotImplementedException("MIlluminance is now called IlluminanceMsg");
    }

    [Obsolete("MImage is now called ImageMsg")]
    public class MImage : ImageMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MImage() => throw new NotImplementedException("MImage is now called ImageMsg");
        public MImage(MHeader header, uint height, uint width, string encoding, byte is_bigendian, uint step, byte[] data) => throw new NotImplementedException("MImage is now called ImageMsg");
    }

    [Obsolete("MImu is now called ImuMsg")]
    public class MImu : ImuMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MImu() => throw new NotImplementedException("MImu is now called ImuMsg");
        public MImu(MHeader header, Geometry.MQuaternion orientation, double[] orientation_covariance, Geometry.MVector3 angular_velocity, double[] angular_velocity_covariance, Geometry.MVector3 linear_acceleration, double[] linear_acceleration_covariance) => throw new NotImplementedException("MImu is now called ImuMsg");
    }

    [Obsolete("MJointState is now called JointStateMsg")]
    public class MJointState : JointStateMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MJointState() => throw new NotImplementedException("MJointState is now called JointStateMsg");
        public MJointState(MHeader header, string[] name, double[] position, double[] velocity, double[] effort) => throw new NotImplementedException("MJointState is now called JointStateMsg");
    }

    [Obsolete("MJoy is now called JoyMsg")]
    public class MJoy : JoyMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MJoy() => throw new NotImplementedException("MJoy is now called JoyMsg");
        public MJoy(MHeader header, float[] axes, int[] buttons) => throw new NotImplementedException("MJoy is now called JoyMsg");
    }

    [Obsolete("MJoyFeedback is now called JoyFeedbackMsg")]
    public class MJoyFeedback : JoyFeedbackMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MJoyFeedback() => throw new NotImplementedException("MJoyFeedback is now called JoyFeedbackMsg");
        public MJoyFeedback(byte type, byte id, float intensity) => throw new NotImplementedException("MJoyFeedback is now called JoyFeedbackMsg");
    }

    [Obsolete("MJoyFeedbackArray is now called JoyFeedbackArrayMsg")]
    public class MJoyFeedbackArray : JoyFeedbackArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MJoyFeedbackArray() => throw new NotImplementedException("MJoyFeedbackArray is now called JoyFeedbackArrayMsg");
        public MJoyFeedbackArray(MJoyFeedback[] array) => throw new NotImplementedException("MJoyFeedbackArray is now called JoyFeedbackArrayMsg");
    }

    [Obsolete("MLaserEcho is now called LaserEchoMsg")]
    public class MLaserEcho : LaserEchoMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MLaserEcho() => throw new NotImplementedException("MLaserEcho is now called LaserEchoMsg");
        public MLaserEcho(float[] echoes) => throw new NotImplementedException("MLaserEcho is now called LaserEchoMsg");
    }

    [Obsolete("MLaserScan is now called LaserScanMsg")]
    public class MLaserScan : LaserScanMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MLaserScan() => throw new NotImplementedException("MLaserScan is now called LaserScanMsg");
        public MLaserScan(MHeader header, float angle_min, float angle_max, float angle_increment, float time_increment, float scan_time, float range_min, float range_max, float[] ranges, float[] intensities) => throw new NotImplementedException("MLaserScan is now called LaserScanMsg");
    }

    [Obsolete("MMagneticField is now called MagneticFieldMsg")]
    public class MMagneticField : MagneticFieldMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMagneticField() => throw new NotImplementedException("MMagneticField is now called MagneticFieldMsg");
        public MMagneticField(MHeader header, Geometry.MVector3 magnetic_field, double[] magnetic_field_covariance) => throw new NotImplementedException("MMagneticField is now called MagneticFieldMsg");
    }

    [Obsolete("MMultiDOFJointState is now called MultiDOFJointStateMsg")]
    public class MMultiDOFJointState : MultiDOFJointStateMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMultiDOFJointState() => throw new NotImplementedException("MMultiDOFJointState is now called MultiDOFJointStateMsg");
        public MMultiDOFJointState(MHeader header, string[] joint_names, Geometry.MTransform[] transforms, Geometry.MTwist[] twist, Geometry.MWrench[] wrench) => throw new NotImplementedException("MMultiDOFJointState is now called MultiDOFJointStateMsg");
    }

    [Obsolete("MMultiEchoLaserScan is now called MultiEchoLaserScanMsg")]
    public class MMultiEchoLaserScan : MultiEchoLaserScanMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMultiEchoLaserScan() => throw new NotImplementedException("MMultiEchoLaserScan is now called MultiEchoLaserScanMsg");
        public MMultiEchoLaserScan(MHeader header, float angle_min, float angle_max, float angle_increment, float time_increment, float scan_time, float range_min, float range_max, MLaserEcho[] ranges, MLaserEcho[] intensities) => throw new NotImplementedException("MMultiEchoLaserScan is now called MultiEchoLaserScanMsg");
    }

    [Obsolete("MNavSatFix is now called NavSatFixMsg")]
    public class MNavSatFix : NavSatFixMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MNavSatFix() => throw new NotImplementedException("MNavSatFix is now called NavSatFixMsg");
        public MNavSatFix(MHeader header, MNavSatStatus status, double latitude, double longitude, double altitude, double[] position_covariance, byte position_covariance_type) => throw new NotImplementedException("MNavSatFix is now called NavSatFixMsg");
    }

    [Obsolete("MNavSatStatus is now called NavSatStatusMsg")]
    public class MNavSatStatus : NavSatStatusMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MNavSatStatus() => throw new NotImplementedException("MNavSatStatus is now called NavSatStatusMsg");
        public MNavSatStatus(sbyte status, ushort service) => throw new NotImplementedException("MNavSatStatus is now called NavSatStatusMsg");
    }

    [Obsolete("MPointCloud is now called PointCloudMsg")]
    public class MPointCloud : PointCloudMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPointCloud() => throw new NotImplementedException("MPointCloud is now called PointCloudMsg");
        public MPointCloud(MHeader header, Geometry.MPoint32[] points, MChannelFloat32[] channels) => throw new NotImplementedException("MPointCloud is now called PointCloudMsg");
    }

    [Obsolete("MPointCloud2 is now called PointCloud2Msg")]
    public class MPointCloud2 : PointCloud2Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPointCloud2() => throw new NotImplementedException("MPointCloud2 is now called PointCloud2Msg");
        public MPointCloud2(MHeader header, uint height, uint width, MPointField[] fields, bool is_bigendian, uint point_step, uint row_step, byte[] data, bool is_dense) => throw new NotImplementedException("MPointCloud2 is now called PointCloud2Msg");
    }

    [Obsolete("MPointField is now called PointFieldMsg")]
    public class MPointField : PointFieldMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPointField() => throw new NotImplementedException("MPointField is now called PointFieldMsg");
        public MPointField(string name, uint offset, byte datatype, uint count) => throw new NotImplementedException("MPointField is now called PointFieldMsg");
    }

    [Obsolete("MRange is now called RangeMsg")]
    public class MRange : RangeMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MRange() => throw new NotImplementedException("MRange is now called RangeMsg");
        public MRange(MHeader header, byte radiation_type, float field_of_view, float min_range, float max_range, float range) => throw new NotImplementedException("MRange is now called RangeMsg");
    }

    [Obsolete("MRegionOfInterest is now called RegionOfInterestMsg")]
    public class MRegionOfInterest : RegionOfInterestMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MRegionOfInterest() => throw new NotImplementedException("MRegionOfInterest is now called RegionOfInterestMsg");
        public MRegionOfInterest(uint x_offset, uint y_offset, uint height, uint width, bool do_rectify) => throw new NotImplementedException("MRegionOfInterest is now called RegionOfInterestMsg");
    }

    [Obsolete("MRelativeHumidity is now called RelativeHumidityMsg")]
    public class MRelativeHumidity : RelativeHumidityMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MRelativeHumidity() => throw new NotImplementedException("MRelativeHumidity is now called RelativeHumidityMsg");
        public MRelativeHumidity(MHeader header, double relative_humidity, double variance) => throw new NotImplementedException("MRelativeHumidity is now called RelativeHumidityMsg");
    }

    [Obsolete("MTemperature is now called TemperatureMsg")]
    public class MTemperature : TemperatureMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTemperature() => throw new NotImplementedException("MTemperature is now called TemperatureMsg");
        public MTemperature(MHeader header, double temperature, double variance) => throw new NotImplementedException("MTemperature is now called TemperatureMsg");
    }

    [Obsolete("MTimeReference is now called TimeReferenceMsg")]
    public class MTimeReference : TimeReferenceMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MTimeReference() => throw new NotImplementedException("MTimeReference is now called TimeReferenceMsg");
        public MTimeReference(MHeader header, MTime time_ref, string source) => throw new NotImplementedException("MTimeReference is now called TimeReferenceMsg");
    }

    [Obsolete("MSetCameraInfoRequest is now called SetCameraInfoRequest")]
    public class MSetCameraInfoRequest : SetCameraInfoRequest
    {
        public new const string RosMessageName = k_RosMessageName;
        public MSetCameraInfoRequest() => throw new NotImplementedException("MSetCameraInfoRequest is now called SetCameraInfoRequest");
        public MSetCameraInfoRequest(MCameraInfo camera_info) => throw new NotImplementedException("MSetCameraInfoRequest is now called SetCameraInfoRequest");
    }

    [Obsolete("MSetCameraInfoResponse is now called SetCameraInfoResponse")]
    public class MSetCameraInfoResponse : SetCameraInfoResponse
    {
        public new const string RosMessageName = k_RosMessageName;
        public MSetCameraInfoResponse() => throw new NotImplementedException("MSetCameraInfoResponse is now called SetCameraInfoResponse");
        public MSetCameraInfoResponse(bool success, string status_message) => throw new NotImplementedException("MSetCameraInfoResponse is now called SetCameraInfoResponse");
    }
}

namespace RosMessageTypes.Shape
{
    [Obsolete("MMesh is now called MeshMsg")]
    public class MMesh : MeshMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMesh() => throw new NotImplementedException("MMesh is now called MeshMsg");
        public MMesh(MMeshTriangle[] triangles, Geometry.MPoint[] vertices) => throw new NotImplementedException("MMesh is now called MeshMsg");
    }

    [Obsolete("MMeshTriangle is now called MeshTriangleMsg")]
    public class MMeshTriangle : MeshTriangleMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMeshTriangle() => throw new NotImplementedException("MMeshTriangle is now called MeshTriangleMsg");
        public MMeshTriangle(uint[] vertex_indices) => throw new NotImplementedException("MMeshTriangle is now called MeshTriangleMsg");
    }

    [Obsolete("MPlane is now called PlaneMsg")]
    public class MPlane : PlaneMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MPlane() => throw new NotImplementedException("MPlane is now called PlaneMsg");
        public MPlane(double[] coef) => throw new NotImplementedException("MPlane is now called PlaneMsg");
    }

    [Obsolete("MSolidPrimitive is now called SolidPrimitiveMsg")]
    public class MSolidPrimitive : SolidPrimitiveMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MSolidPrimitive() => throw new NotImplementedException("MSolidPrimitive is now called SolidPrimitiveMsg");
        public MSolidPrimitive(byte type, double[] dimensions) => throw new NotImplementedException("MSolidPrimitive is now called SolidPrimitiveMsg");
    }
}

namespace RosMessageTypes.Std
{
    [Obsolete("MBool is now called BoolMsg")]
    public class MBool : BoolMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MBool() => throw new NotImplementedException("MBool is now called BoolMsg");
        public MBool(bool data) => throw new NotImplementedException("MBool is now called BoolMsg");
    }

    [Obsolete("MByte is now called ByteMsg")]
    public class MByte : ByteMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MByte() => throw new NotImplementedException("MByte is now called ByteMsg");
        public MByte(sbyte data) => throw new NotImplementedException("MByte is now called ByteMsg");
    }

    [Obsolete("MByteMultiArray is now called ByteMultiArrayMsg")]
    public class MByteMultiArray : ByteMultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MByteMultiArray() => throw new NotImplementedException("MByteMultiArray is now called ByteMultiArrayMsg");
        public MByteMultiArray(MMultiArrayLayout layout, sbyte[] data) => throw new NotImplementedException("MByteMultiArray is now called ByteMultiArrayMsg");
    }

    [Obsolete("MChar is now called CharMsg")]
    public class MChar : CharMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MChar() => throw new NotImplementedException("MChar is now called CharMsg");
        public MChar(byte data) => throw new NotImplementedException("MChar is now called CharMsg");
    }

    [Obsolete("MColorRGBA is now called ColorRGBAMsg")]
    public class MColorRGBA : ColorRGBAMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MColorRGBA() => throw new NotImplementedException("MColorRGBA is now called ColorRGBAMsg");
        public MColorRGBA(float r, float g, float b, float a) => throw new NotImplementedException("MColorRGBA is now called ColorRGBAMsg");
    }

    [Obsolete("MDuration is now called DurationMsg")]
    public class MDuration : DurationMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public int secs;
        public int nsecs;
        public MDuration() => throw new NotImplementedException("MDuration is now called DurationMsg");
        public MDuration(int secs, int nsecs) => throw new NotImplementedException("MDuration is now called DurationMsg");
    }

    [Obsolete("MEmpty is now called EmptyMsg")]
    public class MEmpty : EmptyMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MEmpty() => throw new NotImplementedException("MEmpty is now called EmptyMsg");
    }

    [Obsolete("MFloat32 is now called Float32Msg")]
    public class MFloat32 : Float32Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MFloat32() => throw new NotImplementedException("MFloat32 is now called Float32Msg");
        public MFloat32(float data) => throw new NotImplementedException("MFloat32 is now called Float32Msg");
    }

    [Obsolete("MFloat32MultiArray is now called Float32MultiArrayMsg")]
    public class MFloat32MultiArray : Float32MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MFloat32MultiArray() => throw new NotImplementedException("MFloat32MultiArray is now called Float32MultiArrayMsg");
        public MFloat32MultiArray(MMultiArrayLayout layout, float[] data) => throw new NotImplementedException("MFloat32MultiArray is now called Float32MultiArrayMsg");
    }

    [Obsolete("MFloat64 is now called Float64Msg")]
    public class MFloat64 : Float64Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MFloat64() => throw new NotImplementedException("MFloat64 is now called Float64Msg");
        public MFloat64(double data) => throw new NotImplementedException("MFloat64 is now called Float64Msg");
    }

    [Obsolete("MFloat64MultiArray is now called Float64MultiArrayMsg")]
    public class MFloat64MultiArray : Float64MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MFloat64MultiArray() => throw new NotImplementedException("MFloat64MultiArray is now called Float64MultiArrayMsg");
        public MFloat64MultiArray(MMultiArrayLayout layout, double[] data) => throw new NotImplementedException("MFloat64MultiArray is now called Float64MultiArrayMsg");
    }

    [Obsolete("MHeader is now called HeaderMsg")]
    public class MHeader : HeaderMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MHeader() => throw new NotImplementedException("MHeader is now called HeaderMsg");
        public MHeader(uint seq, MTime stamp, string frame_id) => throw new NotImplementedException("MHeader is now called HeaderMsg");
    }

    [Obsolete("MInt16 is now called Int16Msg")]
    public class MInt16 : Int16Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInt16() => throw new NotImplementedException("MInt16 is now called Int16Msg");
        public MInt16(short data) => throw new NotImplementedException("MInt16 is now called Int16Msg");
    }

    [Obsolete("MInt16MultiArray is now called Int16MultiArrayMsg")]
    public class MInt16MultiArray : Int16MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInt16MultiArray() => throw new NotImplementedException("MInt16MultiArray is now called Int16MultiArrayMsg");
        public MInt16MultiArray(MMultiArrayLayout layout, short[] data) => throw new NotImplementedException("MInt16MultiArray is now called Int16MultiArrayMsg");
    }

    [Obsolete("MInt32 is now called Int32Msg")]
    public class MInt32 : Int32Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInt32() => throw new NotImplementedException("MInt32 is now called Int32Msg");
        public MInt32(int data) => throw new NotImplementedException("MInt32 is now called Int32Msg");
    }

    [Obsolete("MInt32MultiArray is now called Int32MultiArrayMsg")]
    public class MInt32MultiArray : Int32MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInt32MultiArray() => throw new NotImplementedException("MInt32MultiArray is now called Int32MultiArrayMsg");
        public MInt32MultiArray(MMultiArrayLayout layout, int[] data) => throw new NotImplementedException("MInt32MultiArray is now called Int32MultiArrayMsg");
    }

    [Obsolete("MInt64 is now called Int64Msg")]
    public class MInt64 : Int64Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInt64() => throw new NotImplementedException("MInt64 is now called Int64Msg");
        public MInt64(long data) => throw new NotImplementedException("MInt64 is now called Int64Msg");
    }

    [Obsolete("MInt64MultiArray is now called Int64MultiArrayMsg")]
    public class MInt64MultiArray : Int64MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInt64MultiArray() => throw new NotImplementedException("MInt64MultiArray is now called Int64MultiArrayMsg");
        public MInt64MultiArray(MMultiArrayLayout layout, long[] data) => throw new NotImplementedException("MInt64MultiArray is now called Int64MultiArrayMsg");
    }

    [Obsolete("MInt8 is now called Int8Msg")]
    public class MInt8 : Int8Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInt8() => throw new NotImplementedException("MInt8 is now called Int8Msg");
        public MInt8(sbyte data) => throw new NotImplementedException("MInt8 is now called Int8Msg");
    }

    [Obsolete("MInt8MultiArray is now called Int8MultiArrayMsg")]
    public class MInt8MultiArray : Int8MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInt8MultiArray() => throw new NotImplementedException("MInt8MultiArray is now called Int8MultiArrayMsg");
        public MInt8MultiArray(MMultiArrayLayout layout, sbyte[] data) => throw new NotImplementedException("MInt8MultiArray is now called Int8MultiArrayMsg");
    }

    [Obsolete("MMultiArrayDimension is now called MultiArrayDimensionMsg")]
    public class MMultiArrayDimension : MultiArrayDimensionMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMultiArrayDimension() => throw new NotImplementedException("MMultiArrayDimension is now called MultiArrayDimensionMsg");
        public MMultiArrayDimension(string label, uint size, uint stride) => throw new NotImplementedException("MMultiArrayDimension is now called MultiArrayDimensionMsg");
    }

    [Obsolete("MMultiArrayLayout is now called MultiArrayLayoutMsg")]
    public class MMultiArrayLayout : MultiArrayLayoutMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMultiArrayLayout() => throw new NotImplementedException("MMultiArrayLayout is now called MultiArrayLayoutMsg");
        public MMultiArrayLayout(MMultiArrayDimension[] dim, uint data_offset) => throw new NotImplementedException("MMultiArrayLayout is now called MultiArrayLayoutMsg");
    }

    [Obsolete("MString is now called StringMsg")]
    public class MString : StringMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MString() => throw new NotImplementedException("MString is now called StringMsg");
        public MString(string data) => throw new NotImplementedException("MString is now called StringMsg");
    }

    [Obsolete("MTime is now called TimeMsg")]
    public class MTime : TimeMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public uint secs;
        public uint nsecs;
        public MTime() => throw new NotImplementedException("MTime is now called TimeMsg");
        public MTime(uint secs, uint nsecs) => throw new NotImplementedException("MTime is now called TimeMsg");
    }

    [Obsolete("MUInt16 is now called UInt16Msg")]
    public class MUInt16 : UInt16Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MUInt16() => throw new NotImplementedException("MUInt16 is now called UInt16Msg");
        public MUInt16(ushort data) => throw new NotImplementedException("MUInt16 is now called UInt16Msg");
    }

    [Obsolete("MUInt16MultiArray is now called UInt16MultiArrayMsg")]
    public class MUInt16MultiArray : UInt16MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MUInt16MultiArray() => throw new NotImplementedException("MUInt16MultiArray is now called UInt16MultiArrayMsg");
        public MUInt16MultiArray(MMultiArrayLayout layout, ushort[] data) => throw new NotImplementedException("MUInt16MultiArray is now called UInt16MultiArrayMsg");
    }

    [Obsolete("MUInt32 is now called UInt32Msg")]
    public class MUInt32 : UInt32Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MUInt32() => throw new NotImplementedException("MUInt32 is now called UInt32Msg");
        public MUInt32(uint data) => throw new NotImplementedException("MUInt32 is now called UInt32Msg");
    }

    [Obsolete("MUInt32MultiArray is now called UInt32MultiArrayMsg")]
    public class MUInt32MultiArray : UInt32MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MUInt32MultiArray() => throw new NotImplementedException("MUInt32MultiArray is now called UInt32MultiArrayMsg");
        public MUInt32MultiArray(MMultiArrayLayout layout, uint[] data) => throw new NotImplementedException("MUInt32MultiArray is now called UInt32MultiArrayMsg");
    }

    [Obsolete("MUInt64 is now called UInt64Msg")]
    public class MUInt64 : UInt64Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MUInt64() => throw new NotImplementedException("MUInt64 is now called UInt64Msg");
        public MUInt64(ulong data) => throw new NotImplementedException("MUInt64 is now called UInt64Msg");
    }

    [Obsolete("MUInt64MultiArray is now called UInt64MultiArrayMsg")]
    public class MUInt64MultiArray : UInt64MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MUInt64MultiArray() => throw new NotImplementedException("MUInt64MultiArray is now called UInt64MultiArrayMsg");
        public MUInt64MultiArray(MMultiArrayLayout layout, ulong[] data) => throw new NotImplementedException("MUInt64MultiArray is now called UInt64MultiArrayMsg");
    }

    [Obsolete("MUInt8 is now called UInt8Msg")]
    public class MUInt8 : UInt8Msg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MUInt8() => throw new NotImplementedException("MUInt8 is now called UInt8Msg");
        public MUInt8(byte data) => throw new NotImplementedException("MUInt8 is now called UInt8Msg");
    }

    [Obsolete("MUInt8MultiArray is now called UInt8MultiArrayMsg")]
    public class MUInt8MultiArray : UInt8MultiArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MUInt8MultiArray() => throw new NotImplementedException("MUInt8MultiArray is now called UInt8MultiArrayMsg");
        public MUInt8MultiArray(MMultiArrayLayout layout, byte[] data) => throw new NotImplementedException("MUInt8MultiArray is now called UInt8MultiArrayMsg");
    }
}

namespace RosMessageTypes.Stereo
{
    [Obsolete("MDisparityImage is now called DisparityImageMsg")]
    public class MDisparityImage : DisparityImageMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MDisparityImage() => throw new NotImplementedException("MDisparityImage is now called DisparityImageMsg");
        public MDisparityImage(MHeader header, Sensor.MImage image, float f, float T, Sensor.MRegionOfInterest valid_window, float min_disparity, float max_disparity, float delta_d) => throw new NotImplementedException("MDisparityImage is now called DisparityImageMsg");
    }
}

namespace RosMessageTypes.Trajectory
{
    [Obsolete("MJointTrajectory is now called JointTrajectoryMsg")]
    public class MJointTrajectory : JointTrajectoryMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MJointTrajectory() => throw new NotImplementedException("MJointTrajectory is now called JointTrajectoryMsg");
        public MJointTrajectory(MHeader header, string[] joint_names, MJointTrajectoryPoint[] points) => throw new NotImplementedException("MJointTrajectory is now called JointTrajectoryMsg");
    }

    [Obsolete("MJointTrajectoryPoint is now called JointTrajectoryPointMsg")]
    public class MJointTrajectoryPoint : JointTrajectoryPointMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MJointTrajectoryPoint() => throw new NotImplementedException("MJointTrajectoryPoint is now called JointTrajectoryPointMsg");
        public MJointTrajectoryPoint(double[] positions, double[] velocities, double[] accelerations, double[] effort, MDuration time_from_start) => throw new NotImplementedException("MJointTrajectoryPoint is now called JointTrajectoryPointMsg");
    }

    [Obsolete("MMultiDOFJointTrajectory is now called MultiDOFJointTrajectoryMsg")]
    public class MMultiDOFJointTrajectory : MultiDOFJointTrajectoryMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMultiDOFJointTrajectory() => throw new NotImplementedException("MMultiDOFJointTrajectory is now called MultiDOFJointTrajectoryMsg");
        public MMultiDOFJointTrajectory(MHeader header, string[] joint_names, MMultiDOFJointTrajectoryPoint[] points) => throw new NotImplementedException("MMultiDOFJointTrajectory is now called MultiDOFJointTrajectoryMsg");
    }

    [Obsolete("MMultiDOFJointTrajectoryPoint is now called MultiDOFJointTrajectoryPointMsg")]
    public class MMultiDOFJointTrajectoryPoint : MultiDOFJointTrajectoryPointMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMultiDOFJointTrajectoryPoint() => throw new NotImplementedException("MMultiDOFJointTrajectoryPoint is now called MultiDOFJointTrajectoryPointMsg");
        public MMultiDOFJointTrajectoryPoint(Geometry.MTransform[] transforms, Geometry.MTwist[] velocities, Geometry.MTwist[] accelerations, MDuration time_from_start) => throw new NotImplementedException("MMultiDOFJointTrajectoryPoint is now called MultiDOFJointTrajectoryPointMsg");
    }
}

namespace RosMessageTypes.Visualization
{
    [Obsolete("MImageMarker is now called ImageMarkerMsg")]
    public class MImageMarker : ImageMarkerMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MImageMarker() => throw new NotImplementedException("MImageMarker is now called ImageMarkerMsg");
        public MImageMarker(MHeader header, string ns, int id, int type, int action, Geometry.MPoint position, float scale, Std.MColorRGBA outline_color, byte filled, Std.MColorRGBA fill_color, MDuration lifetime, Geometry.MPoint[] points, Std.MColorRGBA[] outline_colors) => throw new NotImplementedException("MImageMarker is now called ImageMarkerMsg");
    }

    [Obsolete("MInteractiveMarker is now called InteractiveMarkerMsg")]
    public class MInteractiveMarker : InteractiveMarkerMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInteractiveMarker() => throw new NotImplementedException("MInteractiveMarker is now called InteractiveMarkerMsg");
        public MInteractiveMarker(MHeader header, Geometry.MPose pose, string name, string description, float scale, MMenuEntry[] menu_entries, MInteractiveMarkerControl[] controls) => throw new NotImplementedException("MInteractiveMarker is now called InteractiveMarkerMsg");
    }

    [Obsolete("MInteractiveMarkerControl is now called InteractiveMarkerControlMsg")]
    public class MInteractiveMarkerControl : InteractiveMarkerControlMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerControl() => throw new NotImplementedException("MInteractiveMarkerControl is now called InteractiveMarkerControlMsg");
        public MInteractiveMarkerControl(string name, Geometry.MQuaternion orientation, byte orientation_mode, byte interaction_mode, bool always_visible, MMarker[] markers, bool independent_marker_orientation, string description) => throw new NotImplementedException("MInteractiveMarkerControl is now called InteractiveMarkerControlMsg");
    }

    [Obsolete("MInteractiveMarkerFeedback is now called InteractiveMarkerFeedbackMsg")]
    public class MInteractiveMarkerFeedback : InteractiveMarkerFeedbackMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerFeedback() => throw new NotImplementedException("MInteractiveMarkerFeedback is now called InteractiveMarkerFeedbackMsg");
        public MInteractiveMarkerFeedback(MHeader header, string client_id, string marker_name, string control_name, byte event_type, Geometry.MPose pose, uint menu_entry_id, Geometry.MPoint mouse_point, bool mouse_point_valid) => throw new NotImplementedException("MInteractiveMarkerFeedback is now called InteractiveMarkerFeedbackMsg");
    }

    [Obsolete("MInteractiveMarkerInit is now called InteractiveMarkerInitMsg")]
    public class MInteractiveMarkerInit : InteractiveMarkerInitMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerInit() => throw new NotImplementedException("MInteractiveMarkerInit is now called InteractiveMarkerInitMsg");
        public MInteractiveMarkerInit(string server_id, ulong seq_num, MInteractiveMarker[] markers) => throw new NotImplementedException("MInteractiveMarkerInit is now called InteractiveMarkerInitMsg");
    }

    [Obsolete("MInteractiveMarkerPose is now called InteractiveMarkerPoseMsg")]
    public class MInteractiveMarkerPose : InteractiveMarkerPoseMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerPose() => throw new NotImplementedException("MInteractiveMarkerPose is now called InteractiveMarkerPoseMsg");
        public MInteractiveMarkerPose(MHeader header, Geometry.MPose pose, string name) => throw new NotImplementedException("MInteractiveMarkerPose is now called InteractiveMarkerPoseMsg");
    }

    [Obsolete("MInteractiveMarkerUpdate is now called InteractiveMarkerUpdateMsg")]
    public class MInteractiveMarkerUpdate : InteractiveMarkerUpdateMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MInteractiveMarkerUpdate() => throw new NotImplementedException("MInteractiveMarkerUpdate is now called InteractiveMarkerUpdateMsg");
        public MInteractiveMarkerUpdate(string server_id, ulong seq_num, byte type, MInteractiveMarker[] markers, MInteractiveMarkerPose[] poses, string[] erases) => throw new NotImplementedException("MInteractiveMarkerUpdate is now called InteractiveMarkerUpdateMsg");
    }

    [Obsolete("MMarker is now called MarkerMsg")]
    public class MMarker : MarkerMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMarker() => throw new NotImplementedException("MMarker is now called MarkerMsg");
        public MMarker(MHeader header, string ns, int id, int type, int action, Geometry.MPose pose, Geometry.MVector3 scale, Std.MColorRGBA color, MDuration lifetime, bool frame_locked, Geometry.MPoint[] points, Std.MColorRGBA[] colors, string text, string mesh_resource, bool mesh_use_embedded_materials) => throw new NotImplementedException("MMarker is now called MarkerMsg");
    }

    [Obsolete("MMarkerArray is now called MarkerArrayMsg")]
    public class MMarkerArray : MarkerArrayMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMarkerArray() => throw new NotImplementedException("MMarkerArray is now called MarkerArrayMsg");
        public MMarkerArray(MMarker[] markers) => throw new NotImplementedException("MMarkerArray is now called MarkerArrayMsg");
    }

    [Obsolete("MMenuEntry is now called MenuEntryMsg")]
    public class MMenuEntry : MenuEntryMsg
    {
        public new const string RosMessageName = k_RosMessageName;
        public MMenuEntry() => throw new NotImplementedException("MMenuEntry is now called MenuEntryMsg");
        public MMenuEntry(uint id, uint parent_id, string title, string command, byte command_type) => throw new NotImplementedException("MMenuEntry is now called MenuEntryMsg");
    }
}
