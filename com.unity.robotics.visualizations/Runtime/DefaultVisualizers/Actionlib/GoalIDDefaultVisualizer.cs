using System;
using RosMessageTypes.Actionlib;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class GoalIDDefaultVisualizer : GuiVisualizer<GoalIDMsg>
    {
        public override Action CreateGUI(GoalIDMsg message, MessageMetadata meta) => () =>
        {
            GUI(message);
        };

        public static void GUI(GoalIDMsg message)
        {
            message.stamp.GUI();
            GUILayout.Label($"ID: {message.id}");
        }
    }
}
