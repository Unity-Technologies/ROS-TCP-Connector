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

        public static Color32 PickColorForTopic(string topic)
        {
            if (topic == "")
                return Color.black;

            byte[] bytes = BitConverter.GetBytes(topic.GetHashCode());
            return new Color32(bytes[0], bytes[1], bytes[2], 255);
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
        public static string TimeToString(MTime message)
        {
            // TODO: display a friendly date/time?
            return $"{message.secs}/{message.nsecs}";
        }

        public static void Draw<C>(this MPoint message, DebugDraw.Drawing drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(this MHeader message)
        {
            GUILayout.Label($"<{message.seq} {message.frame_id} {TimeToString(message.stamp)}>");
        }

        public static void GUI(this MTime message)
        {
            GUILayout.Label(TimeToString(message));
        }

        public static void GUI(this MPoint message, string name)
        {
            string body = $"[{message.x:F2}, {message.y:F2}, {message.z:F2}]";
            if (name == null || name == "")
                GUILayout.Label(body);
            else
                GUILayout.Label($"{name}: {body}");
        }

        public static void GUI(this MPoint message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(this MPoint32 message, DebugDraw.Drawing drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(this MPoint32 message, string name)
        {
            string body = $"[{message.x:F2}, {message.y:F2}, {message.z:F2}]";
            if (name == null || name == "")
                GUILayout.Label(body);
            else
                GUILayout.Label($"{name}: {body}");
        }

        public static void GUI(this MPoint32 message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(this MVector3 message, DebugDraw.Drawing drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(this MVector3 message, string name)
        {
            string body = $"[{message.x:F2}, {message.y:F2}, {message.z:F2}]";
            if (name == null || name == "")
                GUILayout.Label(body);
            else
                GUILayout.Label($"{name}: {body}");
        }

        public static void GUI(this MVector3 message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void GUI(this MPose message)
        {
            message.position.GUI("Position");
            message.orientation.GUI("Orientation");
        }

        public static void GUI(this MPoseArray message)
        {
            GUI(message.header);
            for (int Idx = 0; Idx < message.poses.Length; ++Idx)
            {
                MPose pose = message.poses[Idx];
                pose.position.GUI($"[{Idx}] Position");
                pose.orientation.GUI("Orientation");
            }
        }

        public static void GUIGrid<T>(T[] data, int width)
        {
            int dataIndex = 0;
            while (dataIndex < data.Length)
            {
                GUILayout.BeginHorizontal();
                for (int Idx = 0; Idx < width && dataIndex < data.Length; ++Idx)
                {
                    GUILayout.Label(data[dataIndex].ToString());
                    dataIndex++;
                }
                GUILayout.EndHorizontal();
            }
        }

        public static void Draw<C>(this MPose message, DebugDraw.Drawing drawing, float size = 0.1f) where C : ICoordinateSpace, new()
        {
            DrawAxisVectors(drawing, message.position.From<C>(), message.orientation.From<C>(), size);
        }

        public static void Draw<C>(this MPoseArray message, DebugDraw.Drawing drawing, float size = 0.1f) where C : ICoordinateSpace, new()
        {
            foreach (MPose pose in message.poses)
            {
                pose.Draw<C>(drawing, size);
            }
        }

        public static void Draw<C>(this MQuaternion message, DebugDraw.Drawing drawing, GameObject drawAtPosition = null, float size = 0.1f)
            where C : ICoordinateSpace, new()
        {
            Vector3 position = drawAtPosition != null ? drawAtPosition.transform.position : Vector3.zero;
            DrawAxisVectors(drawing, position, message.From<C>(), size);
        }

        public static void Draw<C>(this MQuaternion message, DebugDraw.Drawing drawing, Vector3 position, float size = 0.1f)
            where C : ICoordinateSpace, new()
        {
            DrawAxisVectors(drawing, position, message.From<C>(), size);
        }

        public static void GUI(this MQuaternion message, string label)
        {
            if (label != "" && label != null)
                label += ": ";
            GUILayout.Label($"{label}[{message.x:F2}, {message.y:F2}, {message.z:F2}, {message.w:F2}]");
        }

        public static void GUI(this MQuaternion message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}, {message.w:F2}]");
        }

        public static void Draw<C>(this MTransform transform, DebugDraw.Drawing drawing, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            transform.rotation.Draw<C>(drawing, transform.translation.From<C>(), size);
        }

        public static void GUI(this MTransform message)
        {
            message.translation.GUI("Translation");
            message.rotation.GUI("Rotation");
        }

        public static void Draw<C>(this MPolygon message, DebugDraw.Drawing drawing, Color color, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 prevPos = message.points[message.points.Length - 1].From<FLU>();
            foreach (MPoint32 p in message.points)
            {
                Vector3 curPos = p.From<C>();
                drawing.DrawLine(prevPos, curPos, color, thickness);
                prevPos = curPos;
            }
        }

        public static void GUI(this MPolygon message)
        {
            GUILayout.Label($"({message.points.Length} points):");
            foreach (MPoint32 p in message.points)
                GUI(p);
        }

        public static void Draw<C>(this MAccel message, DebugDraw.Drawing drawing, Color color, GameObject origin, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 originPos = (origin == null) ? Vector3.zero: origin.transform.position;
            drawing.DrawArrow(originPos, originPos + message.linear.From<C>() * lengthScale, color, thickness);
            DrawAngularVelocityArrow(drawing, message.angular.From<C>(), originPos, color, sphereRadius, thickness);
        }

        public static void GUI(this MAccel message)
        {
            message.linear.GUI("Linear");
            message.angular.GUI("Angular");
        }

        public static void DrawAngularVelocityArrow(DebugDraw.Drawing drawing, Vector3 angularVelocity, Vector3 sphereCenter, Color32 color, float sphereRadius = 1.0f, float arrowThickness = 0.01f)
        {
            DrawRotationArrow(drawing, angularVelocity.normalized, angularVelocity.magnitude * Mathf.Rad2Deg, sphereCenter, color, sphereRadius, arrowThickness);
        }

        public static void DrawRotationArrow(DebugDraw.Drawing drawing, Quaternion rotation, Vector3 sphereCenter, Color32 color, float sphereRadius = 1.0f, float arrowThickness = 0.01f)
        {
            Vector3 axis;
            float angleDegrees;
            rotation.ToAngleAxis(out angleDegrees, out axis);
            DrawRotationArrow(drawing, axis, angleDegrees, sphereCenter, color, sphereRadius, arrowThickness);
        }

        public static void DrawRotationArrow(DebugDraw.Drawing drawing, Vector3 rotationAxis, float rotationDegrees, Vector3 sphereCenter, Color32 color, float sphereRadius = 1.0f, float arrowThickness = 0.01f)
        {
            Vector3 startVector = Vector3.Cross(Vector3.up, rotationAxis);
            if (startVector.sqrMagnitude < 0.01f)
                startVector = Vector3.forward;

            float degreesPerStep = 10.0f; // approximately
            int numSteps = Mathf.Max((int)(rotationDegrees / degreesPerStep), 2);
            float pushOutPerStep = 0;
            if (rotationDegrees > 360)
            {
                float pushOutFinal = arrowThickness * 6 * rotationDegrees / 360;
                pushOutPerStep = pushOutFinal / numSteps;
            }

            Quaternion deltaRotation = Quaternion.AngleAxis(rotationDegrees / (float)numSteps, rotationAxis);
            List<Vector3> points = new List<Vector3>();

            Quaternion currentRotation = Quaternion.LookRotation(startVector);
            for (int step = 0; step < numSteps; ++step)
            {
                points.Add( sphereCenter + currentRotation * Vector3.forward * sphereRadius );
                currentRotation = deltaRotation * currentRotation;
                sphereRadius += pushOutPerStep;
            }

            drawing.DrawLineStrip(color, arrowThickness, points.ToArray());
            drawing.DrawArrow(points[points.Count - 1], sphereCenter + currentRotation * Vector3.forward * sphereRadius, color, arrowThickness);
        }
    }
}