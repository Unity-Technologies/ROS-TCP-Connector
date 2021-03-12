using RosMessageTypes.Moveit;
using RosMessageTypes.NiryoMoveit;
using RosSharp.Urdf;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class PickAndPlaceVisualizerExample : BasicVisualizer<MMoverServiceResponse>
{
    public Color ghostColor;
    public float thickness = 0.01f;
    public float labelSpacing = 0.1f;
    public UrdfRobot forRobot;
    RobotVisualization robotVisualization;

    public override void Start()
    {
        base.Start();
        robotVisualization = new RobotVisualization(forRobot);
    }

    public override void Draw(DebugDraw.Drawing drawing, MMoverServiceResponse message, MessageMetadata meta, Color color, string label)
    {
        int Idx = 1;
        foreach(MRobotTrajectory trajectory in message.trajectories)
        {
            RobotVisualization.JointPlacement[][] jointPlacements = robotVisualization.GetJointPlacements(trajectory.joint_trajectory);
            RobotVisualization.JointPlacement[] finalPose = jointPlacements[jointPlacements.Length - 1];

            robotVisualization.DrawJointPaths(drawing, jointPlacements, color, thickness);
            robotVisualization.DrawGhost(drawing, finalPose, ghostColor);

            drawing.DrawLabel(Idx.ToString(), finalPose[finalPose.Length - 1].Position, color, labelSpacing);
            ++Idx;
        }
    }
}