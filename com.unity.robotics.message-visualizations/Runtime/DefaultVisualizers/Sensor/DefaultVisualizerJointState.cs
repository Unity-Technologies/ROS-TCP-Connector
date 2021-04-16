using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerJointState : BasicVisualizer<MJointState>
{
    public override Action CreateGUI(MJointState message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        for (int i = 0; i < message.name.Length; i++)
        {
            GUILayout.Label($"Name: {message.name[i]}\nPosition: {message.position[i]}\nVelocity: {message.velocity[i]}\nEffort: {message.effort[i]}");
        }
    };
}
