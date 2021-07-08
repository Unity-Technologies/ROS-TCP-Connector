using System;
using RosMessageTypes.Actionlib;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerGoalID : GuiVisualFactory<GoalIDMsg>
    {
        public override Action CreateGUI(GoalIDMsg message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
