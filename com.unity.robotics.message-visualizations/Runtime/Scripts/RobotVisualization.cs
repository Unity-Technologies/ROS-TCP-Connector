using RosMessageTypes.Sensor;
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
        public struct JointPlacement
        {
            public UrdfJoint Joint;
            public Vector3 Position;
            public Quaternion Rotation;
        }

        Dictionary<string, JointPlacement> m_JointsByName = new Dictionary<string, JointPlacement>();
        UrdfRobot m_Robot;

        public RobotVisualization(UrdfRobot robot)
        {
            this.m_Robot = robot;
            foreach (UrdfJoint joint in robot.gameObject.GetComponentsInChildren<UrdfJoint>())
            {
                if (!m_JointsByName.ContainsKey(joint.jointName))
                {
                    m_JointsByName.Add(
                        joint.jointName,
                        new JointPlacement { Joint = joint, Position = joint.transform.localPosition, Rotation = joint.transform.localRotation }
                    );
                }
            }
        }

        // Returns the positions of the selected joint across the whole trajectory
        public Vector3[] GetJointPath(int jointIndex, string[] jointNames, MJointTrajectoryPoint[] point)
        {
            Vector3[] result = new Vector3[point.Length];
            for (int Idx = 0; Idx < point.Length; ++Idx)
            {
                JointPlacement[] placements = GetJointPlacements(point[Idx], jointNames);
                result[Idx] = placements[jointIndex].Position;
            }
            return result;
        }

        // Returns a 2d array: JointPlacement[index of JointTrajectoryPoint][index of joint]
        public JointPlacement[][] GetJointPlacements(MJointTrajectory trajectory)
        {
            JointPlacement[][] result = new JointPlacement[trajectory.points.Length][];
            for (int Idx = 0; Idx < trajectory.points.Length; ++Idx)
            {
                result[Idx] = GetJointPlacements(trajectory.points[Idx], trajectory.joint_names);
            }
            return result;
        }

        public JointPlacement[] GetJointPlacements(MJointState jointState)
        {
            Quaternion lastWorldRotation = m_Robot.transform.rotation;
            Vector3 lastWorldPosition = m_Robot.transform.position;
            GameObject lastJoint = m_Robot.gameObject;
            JointPlacement[] result = new JointPlacement[jointState.name.Length];

            for (int i = 0; i < jointState.name.Length; ++i)
            {
                JointPlacement jointData = m_JointsByName[jointState.name[i]];
                float rotationDegrees = (float)(jointState.position[i] * Mathf.Rad2Deg);

                ArticulationBody body = jointData.Joint.GetComponent<ArticulationBody>();
                Quaternion jointRotation = body.anchorRotation * Quaternion.Euler(rotationDegrees, 0, 0) * Quaternion.Inverse(body.anchorRotation);
                Quaternion localRotation = jointData.Rotation * jointRotation;
                Vector3 localPosition = lastJoint.transform.InverseTransformPoint(body.transform.position);
                Vector3 worldPosition = lastWorldPosition + lastWorldRotation * localPosition;
                Quaternion worldRotation = lastWorldRotation * localRotation;

                result[i] = new JointPlacement { Joint = jointData.Joint, Position = worldPosition, Rotation = worldRotation };

                lastWorldPosition = worldPosition;
                lastWorldRotation = worldRotation;
                lastJoint = body.gameObject;
            }

            return result;
        }

        public JointPlacement[] GetJointPlacements(MJointTrajectoryPoint point, string[] jointNames)
        {
            Quaternion lastWorldRotation = m_Robot.transform.rotation;
            Vector3 lastWorldPosition = m_Robot.transform.position;
            GameObject lastJoint = m_Robot.gameObject;
            JointPlacement[] result = new JointPlacement[jointNames.Length];

            for (int Idx = 0; Idx < point.positions.Length; ++Idx)
            {
                JointPlacement jointData = m_JointsByName[jointNames[Idx]];
                float rotationDegrees = (float)(point.positions[Idx] * Mathf.Rad2Deg);

                ArticulationBody body = jointData.Joint.GetComponent<ArticulationBody>();
                Quaternion jointRotation = body.anchorRotation * Quaternion.Euler(rotationDegrees, 0, 0) * Quaternion.Inverse(body.anchorRotation);
                Quaternion localRotation = jointData.Rotation * jointRotation;
                Vector3 localPosition = lastJoint.transform.InverseTransformPoint(body.transform.position);
                Vector3 worldPosition = lastWorldPosition + lastWorldRotation * localPosition;
                Quaternion worldRotation = lastWorldRotation * localRotation;

                result[Idx] = new JointPlacement { Joint = jointData.Joint, Position = worldPosition, Rotation = worldRotation };

                lastWorldPosition = worldPosition;
                lastWorldRotation = worldRotation;
                lastJoint = body.gameObject;
            }

            return result;
        }

        public UrdfJoint GetJointByName(string joint)
        {
            JointPlacement result;
            if (!m_JointsByName.TryGetValue(joint, out result))
            {
                return null;
            }

            return result.Joint;
        }

        public void DrawJointPaths(BasicDrawing drawing, MJointTrajectory message, Color color, float pathThickness)
        {
            JointPlacement[][] jointPlacements = GetJointPlacements(message);
            for (int Idx = 0; Idx < message.joint_names.Length; ++Idx)
            {
                DrawJointPath(drawing, jointPlacements, Idx, color, pathThickness);
            }
        }

        public void DrawJointPaths(BasicDrawing drawing, JointPlacement[][] jointPlacements, Color color, float pathThickness)
        {
            for (int pathIdx = 1; pathIdx < jointPlacements.Length; ++pathIdx)
            {
                JointPlacement[] pose1 = jointPlacements[pathIdx-1];
                JointPlacement[] pose2 = jointPlacements[pathIdx];
                for (int jointIdx = 0; jointIdx < pose1.Length; ++jointIdx)
                {
                    drawing.DrawLine(pose1[jointIdx].Position, pose2[jointIdx].Position, color, pathThickness);
                }
            }
        }

        public void DrawJointPath(BasicDrawing drawing, MJointTrajectory message, int jointIndex, Color color, float pathThickness)
        {
            DrawJointPath(drawing, GetJointPlacements(message), jointIndex, color, pathThickness);
        }

        public void DrawGhost(BasicDrawing drawing, MJointTrajectory message, int pointIndex, Color color)
        {
            DrawGhost(drawing, GetJointPlacements(message.points[pointIndex], message.joint_names), color);
        }

        public void DrawGhost(BasicDrawing drawing, MJointTrajectoryPoint message, string[] jointNames, Color color)
        {
            DrawGhost(drawing, GetJointPlacements(message, jointNames), color);
        }

        public void DrawJointPath(BasicDrawing drawing, JointPlacement[][] jointPlacements, int jointIndex, Color color, float pathThickness)
        {
            drawing.DrawLineStrip(jointPlacements.Select(p => p[jointIndex].Position).ToArray(), color, pathThickness);
        }

        public void DrawGhost(BasicDrawing drawing, JointPlacement[] placements, Color color)
        {
            foreach(JointPlacement jointPlacement in placements)
            {
                UrdfJoint joint = jointPlacement.Joint;
                int numChildren = joint.transform.childCount;
                for (int Idx = 0; Idx < numChildren; ++Idx)
                {
                    Transform child = joint.transform.GetChild(Idx);
                    if (child.GetComponent<UrdfJoint>()) // don't want to draw the other joints with this jointPlacement
                        continue;
                    foreach (MeshFilter mfilter in child.GetComponentsInChildren<MeshFilter>())
                    {
                        Vector3 localMeshOffset = jointPlacement.Rotation * joint.transform.InverseTransformPoint(mfilter.transform.position);
                        Quaternion localMeshRotation = Quaternion.Inverse(joint.transform.rotation) * mfilter.transform.rotation;
                        drawing.DrawMesh(
                            mfilter.sharedMesh,
                            jointPlacement.Position + localMeshOffset,
                            jointPlacement.Rotation * localMeshRotation,
                            Vector3.one,
                            color
                        );
                    }
                }
            }
        }
    }
}