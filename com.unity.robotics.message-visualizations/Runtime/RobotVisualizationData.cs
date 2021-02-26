using RosSharp.Urdf;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotVisualizationData : MonoBehaviour
{
    Dictionary<string, JointVisualizationData> jointsByName = new Dictionary<string, JointVisualizationData>();

    class JointVisualizationData
    {
        public UrdfJoint joint;
        public Quaternion startRotation;
    }

    public RobotVisualizationData(UrdfRobot robot)
    {
        foreach (UrdfJoint joint in robot.gameObject.GetComponentsInChildren<UrdfJoint>())
        {
            if (!jointsByName.ContainsKey(joint.jointName))
            {
                jointsByName.Add(
                    joint.jointName,
                    new JointVisualizationData { joint = joint, startRotation = joint.transform.localRotation }
                );
            }
        }
    }

    public UrdfJoint GetJointByName(string joint)
    {
        JointVisualizationData result;
        if (!jointsByName.TryGetValue(joint, out result))
        {
            return null;
        }

        return result.joint;
    }

    public UrdfJoint GetJointByName(string joint, out Quaternion startRotation)
    {
        JointVisualizationData result;
        if(!jointsByName.TryGetValue(joint, out result))
        {
            startRotation = Quaternion.identity;
            return null;
        }

        startRotation = result.startRotation;
        return result.joint;
    }
}
