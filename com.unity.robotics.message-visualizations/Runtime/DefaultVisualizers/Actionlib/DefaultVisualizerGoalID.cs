using System;
using RosMessageTypes.Actionlib;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerGoalID : GuiVisualFactory<MGoalID>
    {
        public override Action CreateGUI(MGoalID message, MessageMetadata meta) => () =>
        {
            message.GUI();
        };
    }
}
