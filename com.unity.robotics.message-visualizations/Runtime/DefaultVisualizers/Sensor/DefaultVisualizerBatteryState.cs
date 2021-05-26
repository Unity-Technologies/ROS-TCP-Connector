using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerBatteryState : BasicVisualizer<MBatteryState>
{
    public override Action CreateGUI(MBatteryState message, MessageMetadata meta, BasicDrawing drawing) 
    {
        string status = ((BatteryStateStatusConstants)message.power_supply_status).ToString().Substring("POWER_SUPPLY_STATUS_".Length);
        string health = ((BatteryStateHealthConstants)message.power_supply_health).ToString().Substring("POWER_SUPPLY_HEALTH_".Length);
        string tech = ((BatteryStateTechnologyConstants)message.power_supply_technology).ToString().Substring("POWER_SUPPLY_TECHNOLOGY_".Length);
        string voltage = String.Join(", ", message.cell_voltage);
        string temp = String.Join(", ", message.cell_temperature);

        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Voltage: {message.voltage} (V)\nTemperature: {message.temperature} (ºC)\nCurrent: {message.current} (A)\nCharge: {message.charge} (Ah)\nCapacity: {message.capacity} (Ah)\nDesign Capacity: {message.design_capacity} (Ah)\nPercentage: {message.percentage}");
            GUILayout.Label($"Power supply status: {status}");
            GUILayout.Label($"Power supply health: {health}");
            GUILayout.Label($"Power supply technology: {tech}");
            GUILayout.Label($"Present: {message.present}");
            GUILayout.Label($"Cell voltage: {voltage}");
            GUILayout.Label($"Cell temperature: {temp}");
            GUILayout.Label($"Location: {message.location}\nSerial number: {message.serial_number}");
        };
    }
}
