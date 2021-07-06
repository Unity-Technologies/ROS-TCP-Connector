using System;
using RosMessageTypes.Actionlib;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerGoalStatus : GuiVisualFactory<GoalStatusMsg>
    {
        public override Action CreateGUI(GoalStatusMsg message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
