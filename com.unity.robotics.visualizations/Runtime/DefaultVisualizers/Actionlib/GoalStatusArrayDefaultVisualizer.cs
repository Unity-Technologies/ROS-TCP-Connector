using System;
using RosMessageTypes.Actionlib;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class GoalStatusArrayDefaultVisualizer : GuiVisualizer<GoalStatusArrayMsg>
    {
        public override Action CreateGUI(GoalStatusArrayMsg message, MessageMetadata meta) => () =>
        {
            GUI(message);
        };

        public static void GUI(GoalStatusArrayMsg message)
        {
            message.header.GUI();
            foreach (GoalStatusMsg status in message.status_list)
                GoalStatusDefaultVisualizer.GUI(status);
        }
    }
}
