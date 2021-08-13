using System;
using RosMessageTypes.Diagnostic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DiagnosticArrayDefaultVisualizer : GuiVisualizer<DiagnosticArrayMsg>
    {
        public override Action CreateGUI(DiagnosticArrayMsg message, MessageMetadata meta) => () =>
        {
            GUI(message);
        };

        public static void GUI(DiagnosticArrayMsg message)
        {
            message.header.GUI();
            foreach (DiagnosticStatusMsg status in message.status)
                DiagnosticStatusDefaultVisualizer.GUI(status);
        }
    }
}
