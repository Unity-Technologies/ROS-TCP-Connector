using System;
using RosMessageTypes.Diagnostic;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class DiagnosticStatusDefaultVisualizer : GuiVisualizer<DiagnosticStatusMsg>
    {
        public override Action CreateGUI(DiagnosticStatusMsg message, MessageMetadata meta) => () =>
        {
            GUI(message);
        };

        public static string[] s_DiagnosticLevelTable = new string[]
        {
            "OK","WARN","ERROR","STALE"
        };

        public static void GUI(DiagnosticStatusMsg message)
        {
            string status = (message.level >= 0 && message.level < s_DiagnosticLevelTable.Length) ? s_DiagnosticLevelTable[message.level] : "INVALID";
            GUILayout.Label(message.hardware_id.Length > 0 ? $"Status of {message.name}|{message.hardware_id}: {status}" : $"Status of {message.name}: {status}");
            GUILayout.Label(message.message);
            foreach (KeyValueMsg keyValue in message.values)
            {
                GUILayout.Label($"   {keyValue.key}: {keyValue.value}");
            }
        }
    }
}
