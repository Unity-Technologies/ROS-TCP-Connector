using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerBatteryState : BasicVisualizer<MBatteryState>
{
    public override Action CreateGUI(MBatteryState message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        GUILayout.Label($"Voltage: {message.voltage} (V)\nTemperature: {message.temperature} (ºC)\nCurrent: {message.current} (A)\nCharge: {message.charge} (Ah)\nCapacity: {message.capacity} (Ah)\nDesign Capacity: {message.design_capacity} (Ah)\nPercentage: {message.percentage}");
        GUILayout.Label($"Power supply status: {(BatteryStateStatusConstants)message.power_supply_status}");
        GUILayout.Label($"Power supply health: {(BatteryStateHealthConstants)message.power_supply_health}");
        GUILayout.Label($"Power supply technology: {(BatteryStateTechnologyConstants)message.power_supply_technology}");
        GUILayout.Label($"Present: {message.present}");
        GUILayout.Label($"Cell voltage: {String.Join(", ", message.cell_voltage)}");
        GUILayout.Label($"Cell temperature: {String.Join(", ", message.cell_temperature)}");
        GUILayout.Label($"Location: {message.location}\nSerial number: {message.serial_number}");
    };
}
