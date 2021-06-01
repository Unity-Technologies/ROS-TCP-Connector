using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerNavSatFix : BasicVisualizer<MNavSatFix>
{
    public override Action CreateGUI(MNavSatFix message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.status.GUI();
        GUILayout.Label($"Coordinates: {message.ToLatLongString()}\nAltitude: {message.altitude} (m)\nPosition covariance: {String.Join(", ", message.position_covariance)} (m^2)\nPosition covariance type: {(NavSatFix_Covariance_Constants)message.position_covariance_type}");
    };
}
