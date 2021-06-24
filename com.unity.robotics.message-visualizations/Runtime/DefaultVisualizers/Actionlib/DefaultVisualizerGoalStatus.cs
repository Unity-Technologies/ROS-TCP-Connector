using System;
using RosMessageTypes.Actionlib;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerGoalStatus : GuiVisualFactory<MGoalStatus>
    {
        public override Action CreateGUI(MGoalStatus message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
