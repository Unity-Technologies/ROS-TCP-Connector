using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Trajectory;

namespace Unity.Robotics.MessageVisualizers
{
    public static class MessageVisualizations
    {
        public static Action CreateDefaultGUI(Message message, MessageMetadata meta)
        {
            string text = message.ToString();
            return () => { GUILayout.Label(text); };
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, MPoint message, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(MHeader message)
        {
            GUILayout.Label($"<{message.seq} {message.frame_id} {TimeToString(message.stamp)}>");
        }

        public static void GUI(MTime message)
        {
            GUILayout.Label(TimeToString(message));
        }

        public static string TimeToString(MTime message)
        {
            // TODO: display a friendly date/time?
            return $"{message.secs}/{message.nsecs}";
        }

        public static void GUI(string name, MPoint message)
        {
            string body = $"[{message.x:F2}, {message.y:F2}, {message.z:F2}]";
            if (name == null || name == "")
                GUILayout.Label(body);
            else
                GUILayout.Label($"{name}: {body}");
        }

        public static void GUI(MPoint message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, MPoint32 message, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(string name, MPoint32 message)
        {
            string body = $"[{message.x:F2}, {message.y:F2}, {message.z:F2}]";
            if (name == null || name == "")
                GUILayout.Label(body);
            else
                GUILayout.Label($"{name}: {body}");
        }

        public static void GUI(MPoint32 message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, MVector3 message, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(string name, MVector3 message)
        {
            string body = $"[{message.x:F2}, {message.y:F2}, {message.z:F2}]";
            if (name == null || name == "")
                GUILayout.Label(body);
            else
                GUILayout.Label($"{name}: {body}");
        }

        public static void GUI(MVector3 message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void GUI(MPose message)
        {
            GUI("Position", message.position);
            GUI("Orientation", message.orientation);
        }

        public static void GUI(MPoseArray message)
        {
            for(int Idx = 0; Idx < message.poses.Length; ++Idx)
            {
                MPose pose = message.poses[Idx];
                GUI($"[{Idx}] Position", pose.position);
                GUI("Orientation", pose.orientation);
            }
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, MPose message, float size = 0.1f) where C : ICoordinateSpace, new()
        {
            DrawAxisVectors(drawing, message.position.From<C>(), message.orientation.From<C>(), size);
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, MPoseArray message, float size = 0.1f) where C : ICoordinateSpace, new()
        {
            foreach(MPose pose in message.poses)
            {
                Draw<C>(drawing, pose, size);
            }
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, MQuaternion message, GameObject drawAtPosition = null, float size = 0.1f)
            where C : ICoordinateSpace, new()
        {
            Vector3 position = drawAtPosition != null ? drawAtPosition.transform.position : Vector3.zero;
            DrawAxisVectors(drawing, position, message.From<C>(), size);
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, MQuaternion message, Vector3 position, float size = 0.1f)
            where C : ICoordinateSpace, new()
        {
            DrawAxisVectors(drawing, position, message.From<C>(), size);
        }

        public static void DrawAxisVectors(DebugDraw.Drawing drawing, Vector3 position, Quaternion rotation, float size = 0.1f)
        {
            UnityEngine.Vector3 right = rotation * Vector3.right * size;
            UnityEngine.Vector3 up = rotation * Vector3.up * size;
            UnityEngine.Vector3 forward = rotation * Vector3.forward * size;
            drawing.DrawLine(position, position + right, Color.red, size * 0.1f);
            drawing.DrawLine(position, position + up, Color.green, size * 0.1f);
            drawing.DrawLine(position, position + forward, Color.blue, size * 0.1f);
        }

        public static void GUI(string label, MQuaternion message)
        {
            if (label != "" && label != null)
                label += ": ";
            GUILayout.Label($"{label}[{message.x:F2}, {message.y:F2}, {message.z:F2}, {message.w:F2}]");
        }

        public static void GUI(MQuaternion message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}, {message.w:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, MTransform transform, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            Draw<C>(drawing, transform.rotation, transform.translation.From<C>(), size);
        }

        public static void GUI(MTransform message)
        {
            GUI("Translation", message.translation);
            GUI("Rotation", message.rotation);
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, MPolygon message, Color color, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 prevPos = message.points[message.points.Length - 1].From<FLU>();
            foreach (MPoint32 p in message.points)
            {
                Vector3 curPos = p.From<C>();
                drawing.DrawLine(prevPos, curPos, color, thickness);
                prevPos = curPos;
            }
        }

        public static void GUI(MPolygon message)
        {
            GUILayout.Label($"({message.points.Length} points):");
            foreach(MPoint32 p in message.points)
                GUI(p);
        }

        public static void Draw(DebugDraw.Drawing drawing, string[] joint_names, MJointTrajectoryPoint[] points, Color color)
        {

        }

        public static Color32 PickColorForTopic(string topic)
        {
            if (topic == "")
                return Color.black;

            byte[] bytes = BitConverter.GetBytes(topic.GetHashCode());
            return new Color32(bytes[0], bytes[1], bytes[2], 255);
        }
    }
}