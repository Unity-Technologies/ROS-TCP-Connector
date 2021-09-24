using System;
using RosMessageTypes.Actionlib;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class GoalStatusDefaultVisualizer : GuiVisualizer<GoalStatusMsg>
    {
        public override Action CreateGUI(GoalStatusMsg message, MessageMetadata meta) => () =>
        {
            GUI(message);
        };

        public static string[] s_GoalStatusTable = new string[]
        {
            "PENDING","ACTIVE","PREEMPTED","SUCCEEDED","ABORTED","REJECTED","PREEMPTING","RECALLING","RECALLED","LOST"
        };

        public static void GUI(GoalStatusMsg message)
        {
            string status = (message.status >= 0 && message.status < s_GoalStatusTable.Length) ? s_GoalStatusTable[message.status] : $"INVALID({message.status})";
            GUILayout.Label($"Status: {message.goal_id} = {status}");
            GUILayout.Label(message.text);
        }
    }
}
