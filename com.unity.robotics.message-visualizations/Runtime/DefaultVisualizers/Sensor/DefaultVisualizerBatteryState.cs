using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerBatteryState : BasicVisualFactory<MBatteryState>
{
    public override Action CreateGUI(MBatteryState message, MessageMetadata meta) 
    {
        string voltage = String.Join(", ", message.cell_voltage);
        string temp = String.Join(", ", message.cell_temperature);

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Voltage: {message.voltage} (V)\nTemperature: {message.temperature} (ºC)\nCurrent: {message.current} (A)\nCharge: {message.charge} (Ah)\nCapacity: {message.capacity} (Ah)\nDesign Capacity: {message.design_capacity} (Ah)\nPercentage: {message.percentage}");
            GUILayout.Label($"Power supply status: {(BatteryState_Status_Constants)message.power_supply_status}");
            GUILayout.Label($"Power supply health: {(BatteryState_Health_Constants)message.power_supply_health}");
            GUILayout.Label($"Power supply technology: {(BatteryState_Technology_Constants)message.power_supply_technology}");
            GUILayout.Label($"Present: {message.present}");
            GUILayout.Label($"Cell voltage: {voltage}");
            GUILayout.Label($"Cell temperature: {temp}");
            GUILayout.Label($"Location: {message.location}\nSerial number: {message.serial_number}");
        };
    }
}
