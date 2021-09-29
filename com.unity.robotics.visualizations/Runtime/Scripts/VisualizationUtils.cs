using RosMessageTypes.Actionlib;
using RosMessageTypes.Diagnostic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Shape;
using RosMessageTypes.Std;
using RosMessageTypes.Visualization;
using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace Unity.Robotics.Visualizations
{
    public static class VisualizationUtils
    {
        public static Color SelectColor(Color userColor, MessageMetadata meta)
        {
            if (userColor.r == 0 && userColor.g == 0 && userColor.b == 0)
                return PickColorForTopic(meta.Topic);

            if (userColor.a == 0)
                return new Color(userColor.r, userColor.g, userColor.b, 1);

            return userColor;
        }

        public static string SelectLabel(string userLabel, MessageMetadata meta)
        {
            if (string.IsNullOrEmpty(userLabel))
                return meta.Topic;

            return userLabel;
        }

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

        public static Texture2D MakeTexture(int width, int height, Color color)
        {
            Color[] pix = new Color[16 * 16];

            for (int i = 0; i < pix.Length; i++)
                pix[i] = color;

            Texture2D result = new Texture2D(width, height);
            result.SetPixels(pix);
            result.Apply();
            return result;
        }

        public static bool AssertMessageType<T>(Message message, string topic)
        {
            if (!(message is T))
            {
                Debug.LogError($"Topic \"{topic}\": Can't visualize a message of type {message.GetType()}! (expected {typeof(T)}).");
                return false;
            }

            return true;
        }

        public static IVisual GetVisual(string topic, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            RosTopicState topicState = ROSConnection.GetOrCreateInstance().GetTopic(topic);
            if (topicState != null && subtopic == MessageSubtopic.Response)
                topicState = topicState.ServiceResponseTopic;

            if (topicState == null)
                return null;

            IVisualFactory factory = VisualFactoryRegistry.GetVisualFactory(topic, topicState.RosMessageName, subtopic);
            if (factory == null)
                return null;

            return factory.GetOrCreateVisual(topic);
        }

        public static IVisual GetVisual(string topic, string rosMessageName, MessageSubtopic subtopic)
        {
            IVisualFactory factory = VisualFactoryRegistry.GetVisualFactory(topic, rosMessageName, subtopic);
            if (factory == null)
                return null;

            return factory.GetOrCreateVisual(topic);
        }

        public static void DrawAxisVectors<C>(Drawing3d drawing, Vector3Msg position, QuaternionMsg rotation, float size, bool drawUnityAxes) where C : ICoordinateSpace, new()
        {
            Vector3 unityPosition = position.From<C>();
            Quaternion unityRotation = rotation.From<C>();
            Vector3 x, y, z;
            if (drawUnityAxes)
            {
                x = unityRotation * new Vector3(1, 0, 0) * size;
                y = unityRotation * new Vector3(0, 1, 0) * size;
                z = unityRotation * new Vector3(0, 0, 1) * size;
            }
            else
            {
                x = unityRotation * new Vector3<C>(1, 0, 0).toUnity * size;
                y = unityRotation * new Vector3<C>(0, 1, 0).toUnity * size;
                z = unityRotation * new Vector3<C>(0, 0, 1).toUnity * size;
            }
            drawing.DrawLine(unityPosition, unityPosition + x, Color.red, size * 0.1f);
            drawing.DrawLine(unityPosition, unityPosition + y, Color.green, size * 0.1f);
            drawing.DrawLine(unityPosition, unityPosition + z, Color.blue, size * 0.1f);
        }

        public static void DrawPointCloud<C>(PointMsg[] points, Drawing3d drawing, Color color, float radius = 0.01f)
            where C : ICoordinateSpace, new()
        {
            PointCloudDrawing pointCloud = drawing.AddPointCloud(points.Length);
            foreach (PointMsg p in points)
                pointCloud.AddPoint(p.From<C>(), color, radius);
            pointCloud.Bake();
        }

        public static void DrawPointCloud<C>(Point32Msg[] points, Drawing3d drawing, Color color, float radius = 0.01f)
            where C : ICoordinateSpace, new()
        {
            PointCloudDrawing pointCloud = drawing.AddPointCloud(points.Length);
            foreach (Point32Msg p in points)
                pointCloud.AddPoint(p.From<C>(), color, radius);
            pointCloud.Bake();
        }

        public static void DrawAngularVelocityArrow(Drawing3d drawing, Vector3 angularVelocity, Vector3 sphereCenter, Color32 color, float sphereRadius = 1.0f, float arrowThickness = 0.01f)
        {
            DrawRotationArrow(drawing, angularVelocity.normalized, angularVelocity.magnitude * Mathf.Rad2Deg, sphereCenter, color, sphereRadius, arrowThickness);
        }

        public static void DrawRotationArrow(Drawing3d drawing, Quaternion rotation, Vector3 sphereCenter, Color32 color, float sphereRadius = 1.0f, float arrowThickness = 0.01f)
        {
            Vector3 axis;
            float angleDegrees;
            rotation.ToAngleAxis(out angleDegrees, out axis);
            DrawRotationArrow(drawing, axis, angleDegrees, sphereCenter, color, sphereRadius, arrowThickness);
        }

        public static void DrawRotationArrow(Drawing3d drawing, Vector3 rotationAxis, float rotationDegrees, Vector3 sphereCenter, Color32 color, float sphereRadius = 1.0f, float arrowThickness = 0.01f)
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
                points.Add(sphereCenter + currentRotation * Vector3.forward * sphereRadius);
                currentRotation = deltaRotation * currentRotation;
                sphereRadius += pushOutPerStep;
            }

            drawing.DrawLineStrip(points.ToArray(), color, arrowThickness);
            drawing.DrawArrow(points[points.Count - 1], sphereCenter + currentRotation * Vector3.forward * sphereRadius, color, arrowThickness);
        }

        public static void GUI(this AccelMsg message)
        {
            message.linear.GUI("Linear");
            message.angular.GUI("Angular");
        }

        public static void GUI(this ColorRGBAMsg message, bool withText = true)
        {
            Color oldBackgroundColor = UnityEngine.GUI.color;

            GUILayout.BeginHorizontal();
            UnityEngine.GUI.backgroundColor = message.ToUnityColor();
            GUILayout.Box("", s_ColorSwatchStyle);
            UnityEngine.GUI.backgroundColor = oldBackgroundColor;
            if (withText)
                GUILayout.Label($"R{message.r} G{message.g} B{message.b} A{message.a}");
            GUILayout.EndHorizontal();
        }

        public static void GUI(this HeaderMsg message)
        {
#if !ROS2
            GUILayout.Label($"<{message.seq} {message.frame_id} {message.stamp.ToTimestampString()}>");
#else
            GUILayout.Label($"<{message.frame_id} {message.stamp.ToTimestampString()}>");
#endif

        }

        public static void GUITexture(this Texture2D tex)
        {
            // TODO: Rescale/recenter image based on window height/width
            if (tex == null)
                return;

            var origRatio = tex.width / (float)tex.height;
            UnityEngine.GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), tex);
        }

        public static void GUITexture(this Texture2D tex, Material material)
        {
            if (tex == null)
                return;
            var origRatio = tex.width / (float)tex.height;
            Rect textureRect = GUILayoutUtility.GetAspectRect(origRatio);
            if (Event.current.type == EventType.Repaint)
            {
                Graphics.DrawTexture(textureRect, tex, material);
            }
            else
            {
                UnityEngine.GUI.Box(textureRect, tex);
            }
        }

        public static void GUI(this InertiaMsg message)
        {
            GUILayout.Label($"Mass: {message.m}kg");
            message.com.GUI();
            GUILayout.BeginHorizontal();
            GUILayout.Label(message.ixx.ToString());
            GUILayout.Label(message.ixy.ToString());
            GUILayout.Label(message.ixz.ToString());
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(message.ixy.ToString());
            GUILayout.Label(message.iyy.ToString());
            GUILayout.Label(message.iyz.ToString());
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(message.ixz.ToString());
            GUILayout.Label(message.iyz.ToString());
            GUILayout.Label(message.izz.ToString());
            GUILayout.EndHorizontal();
        }

        public static void GUI(this JoyFeedbackMsg message)
        {
            GUILayout.Label($"Type: {(JoyFeedback_Type_Constants)message.type}\nID: {message.id}\nIntensity: {message.intensity}");
        }

        public static void GUI(this MapMetaDataMsg message)
        {
            GUILayout.Label($"Load time: {message.map_load_time.ToTimestampString()}");
            GUILayout.Label($"Resolution: {message.resolution}");
            GUILayout.Label($"Size: {message.width}x{message.height}");
            message.origin.GUI();
        }

        public static void GUI(this MeshMsg message)
        {
            foreach (PointMsg p in message.vertices)
                p.GUI();
            foreach (MeshTriangleMsg tri in message.triangles)
                tri.GUI();
        }

        public static void GUI(this MeshTriangleMsg message)
        {
            string text = "[" + String.Join(", ", message.vertex_indices.Select(i => i.ToString()).ToArray()) + "]";
            GUILayout.Label(text);
        }

        public static void GUI(this NavSatStatusMsg message)
        {
            GUILayout.Label($"Status: {(NavSatStatus_Type_Constants)message.status}\nService: {(NavSatStatus_Service_Constants)message.service}");
        }

        public static void GUI(this PointMsg message, string name)
        {
            string body = $"[{message.x:F2}, {message.y:F2}, {message.z:F2}]";
            if (name == null || name == "")
                GUILayout.Label(body);
            else
                GUILayout.Label($"{name}: {body}");
        }

        public static void GUI(this PointMsg message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void GUI(this Point32Msg message, string name)
        {
            string body = $"[{message.x:F2}, {message.y:F2}, {message.z:F2}]";
            if (name == null || name == "")
                GUILayout.Label(body);
            else
                GUILayout.Label($"{name}: {body}");
        }

        public static void GUI(this Point32Msg message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void GUI(this PointFieldMsg message)
        {
            GUILayout.Label($"Name: {message.name}\nDatatype: {(PointField_Format_Constants)message.datatype}");
            if (message.count > 1)
                GUILayout.Label($"Count: {message.count}");
        }

        public static void GUI(this PolygonMsg message)
        {
            GUILayout.Label($"({message.points.Length} points):");
            foreach (Point32Msg p in message.points)
                GUI(p);
        }

        public static void GUI(this PoseMsg message, string name = "")
        {
            if (name.Length > 0)
            {
                GUILayout.Label(name);
            }
            message.position.GUI("Position");
            message.orientation.GUI("Orientation");
        }

        public static void GUI(this PoseArrayMsg message)
        {
            GUI(message.header);
            for (int Idx = 0; Idx < message.poses.Length; ++Idx)
            {
                PoseMsg pose = message.poses[Idx];
                pose.position.GUI($"[{Idx}] Position");
                pose.orientation.GUI("Orientation");
            }
        }

        public static void GUI(this QuaternionMsg message, string label)
        {
            if (label != "" && label != null)
                label += ": ";
            GUILayout.Label($"{label}[{message.x:F2}, {message.y:F2}, {message.z:F2}, {message.w:F2}]");
        }

        public static void GUI(this QuaternionMsg message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}, {message.w:F2}]");
        }

        public static void GUI(this RegionOfInterestMsg message, Texture2D tex = null)
        {
            if (tex != null)
            {
                var ratio = (float)tex.width / (float)tex.height;
                UnityEngine.GUI.Box(GUILayoutUtility.GetAspectRect(ratio), tex);
            }
            GUILayout.Label($"x_offset: {message.x_offset}\ny_offset: {message.y_offset}\nHeight: {message.height}\nWidth: {message.width}\nDo rectify: {message.do_rectify}");
        }

        public static void GUI(this SelfTestResponse message)
        {
            string pass = message.passed != 0 ? "OK" : "FAIL";
            GUILayout.Label($"Self test {message.id}: {pass}");
            foreach (DiagnosticStatusMsg status in message.status)
                DiagnosticStatusDefaultVisualizer.GUI(status);
        }

        public static void GUI(this SolidPrimitiveMsg message)
        {
            switch (message.type)
            {
                case SolidPrimitiveMsg.BOX:
                    GUILayout.Label($"SolidPrimitive BOX\n[X:{message.dimensions[SolidPrimitiveMsg.BOX_X]}, Y:{message.dimensions[SolidPrimitiveMsg.BOX_Y]}, Z:{message.dimensions[SolidPrimitiveMsg.BOX_Z]}]");
                    break;
                case SolidPrimitiveMsg.SPHERE:
                    GUILayout.Label($"SolidPrimitive SPHERE\nRadius: {message.dimensions[SolidPrimitiveMsg.SPHERE_RADIUS]}");
                    break;
                case SolidPrimitiveMsg.CYLINDER:
                    GUILayout.Label($"SolidPrimitive CYLINDER\nHeight: {message.dimensions[SolidPrimitiveMsg.CYLINDER_HEIGHT]}\nRadius: {message.dimensions[SolidPrimitiveMsg.CYLINDER_RADIUS]}");
                    break;
                case SolidPrimitiveMsg.CONE:
                    GUILayout.Label($"SolidPrimitive CONE\nHeight: {message.dimensions[SolidPrimitiveMsg.CONE_HEIGHT]}\nRadius: {message.dimensions[SolidPrimitiveMsg.CONE_RADIUS]}");
                    break;
                default:
                    GUILayout.Label($"INVALID shape {message.type}!?");
                    break;
            }
        }

        public static void GUI(this TimeMsg message)
        {
            GUILayout.Label(message.ToTimestampString());
        }

        public static void GUI(this TransformMsg message)
        {
            message.translation.GUI("Translation");
            message.rotation.GUI("Rotation");
        }

        public static void GUI(this TwistMsg message, string name = "")
        {
            if (name.Length > 0)
            {
                GUILayout.Label(name);
            }
            message.linear.GUI("Linear");
            message.angular.GUI("Angular");
        }

        public static void GUI(this Vector3Msg message, string name)
        {
            string body = $"[{message.x:F2}, {message.y:F2}, {message.z:F2}]";
            if (name == null || name == "")
                GUILayout.Label(body);
            else
                GUILayout.Label($"{name}: {body}");
        }

        public static void GUI(this Vector3Msg message)
        {
            GUILayout.Label($"[{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void GUI(this WrenchMsg message)
        {
            message.force.GUI("Force");
            message.torque.GUI("Torque");
        }

        public static void GUIGrid<T>(T[] data, int width, ref bool view)
        {
            view = GUILayout.Toggle(view, "View matrix");
            if (!view) return;

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

        public static void GUIGrid<T>(T[] data, int width, string name, ref bool view)
        {
            view = GUILayout.Toggle(view, $"View {name} matrix");
            if (!view) return;

            int dataIndex = 0;
            GUILayout.Label(name);
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

        static readonly GUIStyle s_ColorSwatchStyle = new GUIStyle
        {
            normal = new GUIStyleState { background = Texture2D.whiteTexture },
            fixedWidth = 25,
            fixedHeight = 25
        };

        public static void GUIMultiArray<T>(this MultiArrayLayoutMsg layout, T[] data, ref bool tabulate)
        {
            tabulate = GUILayout.Toggle(tabulate, "Table view");
            GUIMultiArray(layout, data, tabulate);
        }

        static GUIStyle s_ArrayContainerStyle;

        public static void GUIMultiArray<T>(this MultiArrayLayoutMsg layout, T[] data, bool tabulate = true)
        {
            if (s_ArrayContainerStyle == null)
            {
                s_ArrayContainerStyle = UnityEngine.GUI.skin.GetStyle("box");
            }
            GUIMultiArrayLevel(data, layout, layout.data_offset, 0, tabulate);
        }

        static void GUIMultiArrayLevel<T>(T[] data, MultiArrayLayoutMsg layout, uint dataIndex, int depth, bool tabulate)
        {
            uint size = layout.dim[depth].size;
            if (layout.dim.Length > depth + 1)
            {
                uint stride = layout.dim[depth + 1].stride;
                if (depth > 0)
                    GUILayout.BeginVertical(s_ArrayContainerStyle);
                for (int step = 0; step < size; ++step)
                {
                    GUIMultiArrayLevel(data, layout, dataIndex, depth + 1, tabulate);
                    dataIndex += stride;
                }
                if (depth > 0)
                    GUILayout.EndVertical();
            }
            else if (tabulate)
            {
                GUILayout.BeginHorizontal();
                for (int step = 0; step < size; ++step)
                {
                    GUILayout.Label(data[dataIndex].ToString());
                    dataIndex++;
                }
                GUILayout.EndHorizontal();
            }
            else
            {
                GUILayout.BeginVertical(s_ArrayContainerStyle);
                for (int step = 0; step < size; ++step)
                {
                    GUILayout.Label(data[dataIndex].ToString());
                    dataIndex++;
                }
                GUILayout.EndVertical();
            }
        }
    }
}
