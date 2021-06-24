using System;
using RosMessageTypes.Diagnostic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerDiagnosticArray : GuiVisualFactory<MDiagnosticArray>
    {
        public override Action CreateGUI(MDiagnosticArray message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
