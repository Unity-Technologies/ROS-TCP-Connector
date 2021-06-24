using System;
using RosMessageTypes.Actionlib;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerGoalStatusArray : GuiVisualFactory<MGoalStatusArray>
    {
        public override Action CreateGUI(MGoalStatusArray message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
