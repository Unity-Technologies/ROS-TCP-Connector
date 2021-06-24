using System;
using RosMessageTypes.Diagnostic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerDiagnosticStatus : GuiVisualFactory<MDiagnosticStatus>
    {
        public override Action CreateGUI(MDiagnosticStatus message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
