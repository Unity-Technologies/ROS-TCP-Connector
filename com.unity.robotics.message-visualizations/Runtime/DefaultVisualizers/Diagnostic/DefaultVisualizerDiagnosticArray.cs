using System;
using RosMessageTypes.Diagnostic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerDiagnosticArray : GuiVisualFactory<DiagnosticArrayMsg>
    {
        public override Action CreateGUI(DiagnosticArrayMsg message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
