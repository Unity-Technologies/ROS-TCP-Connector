using System;
using RosMessageTypes.Diagnostic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerDiagnosticStatus : GuiVisualFactory<DiagnosticStatusMsg>
    {
        public override Action CreateGUI(DiagnosticStatusMsg message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
