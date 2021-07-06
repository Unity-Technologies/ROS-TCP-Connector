using System;
using RosMessageTypes.Actionlib;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerGoalStatusArray : GuiVisualFactory<GoalStatusArrayMsg>
    {
        public override Action CreateGUI(GoalStatusArrayMsg message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
