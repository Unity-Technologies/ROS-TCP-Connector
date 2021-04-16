using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMultiDOFJointState : BasicVisualizer<MMultiDOFJointState>
{
    public override Action CreateGUI(MMultiDOFJointState message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        for (int i = 0; i < message.joint_names.Length; i++)
        {
            GUILayout.Label($"Name: {message.joint_names[i]}");
            message.transforms[i].GUI();
            message.twist[i].GUI();
            message.wrench[i].GUI();
        }
    };
}
