using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class BatteryStateDefaultVisualizer : GuiVisualizer<BatteryStateMsg>
{
    public override Action CreateGUI(BatteryStateMsg message, MessageMetadata meta)
    {
        var voltage = string.Join(", ", message.cell_voltage);
        var temp = string.Join(", ", message.cell_temperature);

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
