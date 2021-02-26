using RosMessageTypes.Trajectory;
using RosSharp.Urdf;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
        for(int Idx = 0; Idx < point.Length; ++Idx)
        {
            JointPlacement[] placements = GetJointPlacements(jointNames, point[Idx]);
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
            result[Idx] = GetJointPlacements(trajectory.joint_names, trajectory.points[Idx]);
        }
        return result;
    }

    public JointPlacement[] GetJointPlacements(string[] jointNames, MJointTrajectoryPoint point)
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
        if(!jointsByName.TryGetValue(joint, out result))
        {
            startRotation = Quaternion.identity;
            return null;
        }

        startRotation = result.startRotation;
        return result.joint;
    }
}
