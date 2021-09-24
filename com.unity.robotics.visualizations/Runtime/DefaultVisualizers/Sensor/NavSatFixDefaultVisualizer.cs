using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class NavSatFixDefaultVisualizer : GuiVisualizer<NavSatFixMsg>
{
    public override Action CreateGUI(NavSatFixMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            message.status.GUI();
            GUILayout.Label($"Coordinates: {message.ToLatLongString()}\nAltitude: {message.altitude} (m)\nPosition covariance: {string.Join(", ", message.position_covariance)} (m^2)\nPosition covariance type: {(NavSatFix_Covariance_Constants)message.position_covariance_type}");
        };
    }
}
