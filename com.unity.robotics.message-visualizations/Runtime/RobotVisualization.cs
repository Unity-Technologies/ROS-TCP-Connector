using RosMessageTypes.Trajectory;
using RosSharp.Urdf;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class RobotVisualization
    {
        Dictionary<string, JointData> jointsByName = new Dictionary<string, JointData>();

        class JointData
        {
            public UrdfJoint joint;
            public Quaternion startRotation;
        }

        public struct JointPlacement
        {
            public UrdfJoint joint;
            public Vector3 position;
            public Quaternion rotation;
        }

        UrdfRobot robot;

        public RobotVisualization(UrdfRobot robot)
        {
            this.robot = robot;
            foreach (UrdfJoint joint in robot.gameObject.GetComponentsInChildren<UrdfJoint>())
            {
                if (!jointsByName.ContainsKey(joint.jointName))
                {
                    jointsByName.Add(
                        joint.jointName,
                        new JointData { joint = joint, startRotation = joint.transform.localRotation }
                    );
                }
            }
        }

        /// <returns> the positions of the selected joint across the whole trajectory </returns>
        public Vector3[] GetJointPath(int jointIndex, string[] jointNames, MJointTrajectoryPoint[] point)
        {
            Vector3[] result = new Vector3[point.Length];
            for (int Idx = 0; Idx < point.Length; ++Idx)
            {
                JointPlacement[] placements = GetJointPlacements(point[Idx], jointNames);
                result[Idx] = placements[jointIndex].position;
            }
            return result;
        }

        /// <returns> JointPlacement[which MJointTrajectoryPoint in the trajectory][which joint]</returns>
        public JointPlacement[][] GetJointPlacements(MJointTrajectory trajectory)
        {
            JointPlacement[][] result = new JointPlacement[trajectory.points.Length][];
            for (int Idx = 0; Idx < trajectory.points.Length; ++Idx)
            {
                result[Idx] = GetJointPlacements(trajectory.points[Idx], trajectory.joint_names);
            }
            return result;
        }

        public JointPlacement[] GetJointPlacements(MJointTrajectoryPoint point, string[] jointNames)
        {
            Quaternion lastRotation = robot.transform.rotation;
            Vector3 lastWorldPosition = robot.transform.position;
            GameObject lastJoint = robot.gameObject;
            JointPlacement[] result = new JointPlacement[jointNames.Length];

            for (int Idx = 0; Idx < point.positions.Length; ++Idx)
            {
                JointData jointData = jointsByName[jointNames[Idx]];
                float rotationDegrees = (float)(point.positions[Idx] * Mathf.Rad2Deg);

                ArticulationBody body = jointData.joint.GetComponent<ArticulationBody>();
                Quaternion jointRotation = body.anchorRotation * Quaternion.Euler(rotationDegrees, 0, 0) * Quaternion.Inverse(body.anchorRotation);
                Quaternion localRotation = jointData.startRotation * jointRotation;
                Vector3 localPosition = lastJoint.transform.InverseTransformPoint(body.transform.position);
                Vector3 worldPosition = lastWorldPosition + lastRotation * localPosition;
                Quaternion worldRotation = lastRotation * localRotation;
                //drawing.DrawLine(lastWorldPosition, worldPosition, blendedColor, thickness);

                result[Idx] = new JointPlacement { joint = jointData.joint, position = worldPosition, rotation = worldRotation };

                lastWorldPosition = worldPosition;
                lastRotation = worldRotation;
                lastJoint = body.gameObject;

                /*if (isEndPoint)
                {
                    Transform visual = body.transform.Find("Visuals/unnamed");
                    foreach (MeshFilter mfilter in visual.GetComponentsInChildren<MeshFilter>())
                    {
                        Vector3 localMeshOffset = lastRotation * body.transform.InverseTransformPoint(mfilter.transform.position);
                        Quaternion localMeshRotation = Quaternion.Inverse(body.transform.rotation) * mfilter.transform.rotation;
                        drawing.DrawMesh(mfilter.mesh, lastWorldPosition + localMeshOffset, lastRotation * localMeshRotation, Vector3.one, blendedColor);
                    }
                }*/
                //joint.transform.localRotation = localRotation;
                //body.enabled = false;
            }

            return result;
        }

        public UrdfJoint GetJointByName(string joint)
        {
            JointData result;
            if (!jointsByName.TryGetValue(joint, out result))
            {
                return null;
            }

            return result.joint;
        }

        public UrdfJoint GetJointByName(string joint, out Quaternion startRotation)
        {
            JointData result;
            if (!jointsByName.TryGetValue(joint, out result))
            {
                startRotation = Quaternion.identity;
                return null;
            }

            startRotation = result.startRotation;
            return result.joint;
        }

        public void DrawJointPaths(DebugDraw.Drawing drawing, MJointTrajectory message, Color color, float pathThickness)
        {
            JointPlacement[][] jointPlacements = GetJointPlacements(message);
            for (int Idx = 0; Idx < message.joint_names.Length; ++Idx)
            {
                DrawJointPath(drawing, jointPlacements, Idx, color, pathThickness);
            }
        }

        public void DrawJointPaths(DebugDraw.Drawing drawing, JointPlacement[][] jointPlacements, Color color, float pathThickness)
        {
            for (int pathIdx = 1; pathIdx < jointPlacements.Length; ++pathIdx)
            {
                JointPlacement[] pose1 = jointPlacements[pathIdx-1];
                JointPlacement[] pose2 = jointPlacements[pathIdx];
                for (int jointIdx = 0; jointIdx < pose1.Length; ++jointIdx)
                {
                    drawing.DrawLine(pose1[jointIdx].position, pose2[jointIdx].position, color, pathThickness);
                }
            }
        }

        public void DrawJointPath(DebugDraw.Drawing drawing, MJointTrajectory message, int jointIndex, Color color, float pathThickness)
        {
            DrawJointPath(drawing, GetJointPlacements(message), jointIndex, color, pathThickness);
        }

        public void DrawGhost(DebugDraw.Drawing drawing, MJointTrajectory message, int pointIndex, Color color)
        {
            DrawGhost(drawing, GetJointPlacements(message.points[pointIndex], message.joint_names), color);
        }

        public void DrawGhost(DebugDraw.Drawing drawing, MJointTrajectoryPoint message, string[] jointNames, Color color)
        {
            DrawGhost(drawing, GetJointPlacements(message, jointNames), color);
        }


        public void DrawJointPath(DebugDraw.Drawing drawing, JointPlacement[][] jointPlacements, int jointIndex, Color color, float pathThickness)
        {
            drawing.DrawLineStrip(color, pathThickness, jointPlacements.Select(p => p[jointIndex].position).ToArray());
        }

        public void DrawGhost(DebugDraw.Drawing drawing, JointPlacement[] placements, Color color)
        {
            foreach(JointPlacement jointPlacement in placements)
            {
                UrdfJoint joint = jointPlacement.joint;
                int numChildren = joint.transform.childCount;
                for (int Idx = 0; Idx < numChildren; ++Idx)
                {
                    Transform child = joint.transform.GetChild(Idx);
                    if (child.GetComponent<UrdfJoint>()) // don't want to draw the other joints with this jointPlacement
                        continue;
                    foreach (MeshFilter mfilter in child.GetComponentsInChildren<MeshFilter>())
                    {
                        Vector3 localMeshOffset = jointPlacement.rotation * joint.transform.InverseTransformPoint(mfilter.transform.position);
                        Quaternion localMeshRotation = Quaternion.Inverse(joint.transform.rotation) * mfilter.transform.rotation;
                        drawing.DrawMesh(
                            mfilter.sharedMesh,
                            jointPlacement.position + localMeshOffset,
                            jointPlacement.rotation * localMeshRotation,
                            Vector3.one,
                            color
                        );
                    }
                }
            }
        }
    }
}