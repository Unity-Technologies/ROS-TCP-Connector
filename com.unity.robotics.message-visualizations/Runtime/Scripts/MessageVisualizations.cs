using RosMessageTypes.Actionlib;
using RosMessageTypes.Diagnostic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Shape;
using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

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

        public static void DrawAxisVectors<C>(BasicDrawing drawing, MVector3 position, MQuaternion rotation, float size, bool drawUnityAxes) where C : ICoordinateSpace, new()
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

        public static void Draw<C>(this MAccel message, BasicDrawing drawing, Color color, GameObject origin, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 originPos = (origin == null) ? Vector3.zero : origin.transform.position;
            drawing.DrawArrow(originPos, originPos + message.linear.From<C>() * lengthScale, color, thickness);
            DrawAngularVelocityArrow(drawing, message.angular.From<C>(), originPos, color, sphereRadius, thickness);
        }

        public static void Draw<C>(this MGridCells message, BasicDrawing drawing, Color color, float radius = 0.01f) where C : ICoordinateSpace, new()
        {
            DrawPointCloud<C>(message.cells, drawing, color, radius);
        }

        public static void DrawPointCloud<C>(MPoint[] points, BasicDrawing drawing, Color color, float radius = 0.01f) where C : ICoordinateSpace, new()
        {
            PointCloudDrawing pointCloud = drawing.AddPointCloud(points.Length);
            foreach (MPoint p in points)
                pointCloud.AddPoint(p.From<C>(), color, radius);
            pointCloud.Bake();
        }

        public static void Draw<C>(this MMesh message, BasicDrawing drawing, Color color, GameObject origin = null) where C : ICoordinateSpace, new()
        {
            Mesh mesh = new Mesh();
            mesh.vertices = message.vertices.Select(v => v.From<C>()).ToArray();
            mesh.triangles = message.triangles.SelectMany(tri => tri.vertex_indices.Select(i => (int)i)).ToArray();
            if (origin != null)
                drawing.DrawMesh(mesh, origin.transform, color);
            else
                drawing.DrawMesh(mesh, Vector3.zero, Quaternion.identity, Vector3.one, color);
        }

        static Mesh s_OccupancyGridMesh;
        static Material s_OccupancyGridMaterial = new Material(Shader.Find("Unlit/Color"));

        public static void Draw<C>(this MOccupancyGrid message, BasicDrawing drawing) where C : ICoordinateSpace, new()
        {
            Vector3 origin = message.info.origin.position.From<C>();
            Quaternion rotation = message.info.origin.orientation.From<C>();
            int width = (int)message.info.width;
            int height = (int)message.info.height;
            float scale = message.info.resolution;

            if (s_OccupancyGridMesh == null)
            {
                s_OccupancyGridMesh = new Mesh();
                s_OccupancyGridMesh.vertices = new Vector3[] { Vector3.zero, new Vector3(0, 0, 1), new Vector3(1, 0, 1), new Vector3(1, 0, 0) };
                s_OccupancyGridMesh.uv = new Vector2[] { Vector2.zero, Vector2.up, Vector2.one, Vector2.right };
                s_OccupancyGridMesh.triangles = new int[] { 0, 1, 2, 2, 3, 0, };
                s_OccupancyGridMaterial = new Material(Shader.Find("Unlit/OccupancyGrid"));
            }

            Texture2D gridTexture = new Texture2D(width, height, TextureFormat.R8, true);
            gridTexture.wrapMode = TextureWrapMode.Clamp;
            gridTexture.filterMode = FilterMode.Point;
            gridTexture.SetPixelData(message.data, 0);
            gridTexture.Apply();

            Material gridMaterial = new Material(s_OccupancyGridMaterial);
            gridMaterial.mainTexture = gridTexture;

            drawing.DrawMesh(s_OccupancyGridMesh, origin - rotation * new Vector3(scale * 0.5f, 0, scale * 0.5f), rotation, new Vector3(width * scale, 1, height * scale), gridMaterial);
        }

        public static void Draw<C>(this MPath message, BasicDrawing drawing, Color color, float thickness = 0.1f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPath(message.poses.Select(pose => pose.pose.position.From<C>()), color, thickness);
        }

        public static void Draw<C>(this MPlane message, BasicDrawing drawing, Color color, GameObject center = null, float size = 10.0f) where C : ICoordinateSpace, new()
        {
            message.Draw<C>(drawing, color, (center != null) ? center.transform.position : Vector3.zero, size);
        }

        public static void Draw<C>(this MPlane message, BasicDrawing drawing, Color color, Vector3 origin, float size = 10.0f) where C : ICoordinateSpace, new()
        {
            Vector3 normal = new Vector3<C>((float)message.coef[0], (float)message.coef[1], (float)message.coef[2]).toUnity;
            float d = (float)message.coef[3];

            float normalScale = (Vector3.Dot(normal, origin) + d) / normal.sqrMagnitude;
            Vector3 center = origin - normal * normalScale;

            Vector3 forward = (Mathf.Abs(normal.x) > Mathf.Abs(normal.y)) ? Vector3.Cross(normal, Vector3.up).normalized : Vector3.Cross(normal, Vector3.right).normalized;
            Vector3 side = Vector3.Cross(normal, forward).normalized;
            Vector3 diagonalA = (forward + side) * size;
            Vector3 diagonalB = (forward - side) * size;
            drawing.DrawQuad(center - diagonalA, center + diagonalB, center + diagonalA, center - diagonalB, color, true);
        }

        public static void Draw<C>(this MPoint message, BasicDrawing drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void Draw<C>(this MPoint message, BasicDrawing drawing, Color color, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
        }

        public static void Draw<C>(this MPoint32 message, BasicDrawing drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void Draw<C>(this MPoint32 message, BasicDrawing drawing, Color color, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
        }

        public static void Draw<C>(this MPolygon message, BasicDrawing drawing, Color color, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 prevPos = message.points[message.points.Length - 1].From<C>();
            foreach (MPoint32 p in message.points)
            {
                Vector3 curPos = p.From<C>();
                drawing.DrawLine(prevPos, curPos, color, thickness);
                prevPos = curPos;
            }
        }

        public static void Draw<C>(this MPose message, BasicDrawing drawing, float size = 0.1f, bool drawUnityAxes = false) where C : ICoordinateSpace, new()
        {
            DrawAxisVectors<C>(
                drawing,
                new MVector3(message.position.x, message.position.y, message.position.z),
                message.orientation,
                size,
                drawUnityAxes
            );
        }

        public static void Draw<C>(this MPoseArray message, BasicDrawing drawing, float size = 0.1f, bool drawUnityAxes = false) where C : ICoordinateSpace, new()
        {
            foreach (MPose pose in message.poses)
            {
                pose.Draw<C>(drawing, size, drawUnityAxes);
            }
        }

        public static void Draw<C>(this MQuaternion message, BasicDrawing drawing, GameObject drawAtPosition = null, float size = 0.1f, bool drawUnityAxes = false)
    where C : ICoordinateSpace, new()
        {
            Vector3 position = drawAtPosition != null ? drawAtPosition.transform.position : Vector3.zero;
            DrawAxisVectors<C>(drawing, position.To<C>(), message, size, drawUnityAxes);
        }

        public static void Draw<C>(this MQuaternion message, BasicDrawing drawing, Vector3 position, float size = 0.1f, bool drawUnityAxes = false)
            where C : ICoordinateSpace, new()
        {
            DrawAxisVectors<C>(drawing, position.To<C>(), message, size, drawUnityAxes);
        }

        public static void Draw<C>(this MSolidPrimitive message, BasicDrawing drawing, Color color, GameObject origin = null)
            where C : ICoordinateSpace, new()
        {
            Vector3 originPosition = origin != null ? origin.transform.position : Vector3.zero;
            Quaternion originRotation = origin != null ? origin.transform.rotation : Quaternion.identity;
            switch (message.type)
            {
                case MSolidPrimitive.BOX:
                    drawing.DrawCuboid(
                        originPosition,
                        new Vector3<C>(
                            (float)message.dimensions[MSolidPrimitive.BOX_X] * 0.5f,
                            (float)message.dimensions[MSolidPrimitive.BOX_Y] * 0.5f,
                            (float)message.dimensions[MSolidPrimitive.BOX_Z] * 0.5f).toUnity,
                        originRotation,
                        color
                    );
                    break;
                case MSolidPrimitive.SPHERE:
                    drawing.DrawSphere(originPosition, color, (float)message.dimensions[MSolidPrimitive.SPHERE_RADIUS]);
                    break;
                case MSolidPrimitive.CYLINDER:
                    Vector3 cylinderAxis = originRotation * Vector3.up * (float)message.dimensions[MSolidPrimitive.CYLINDER_HEIGHT] * 0.5f;
                    drawing.DrawCylinder(originPosition - cylinderAxis, originPosition + cylinderAxis, color, (float)message.dimensions[MSolidPrimitive.CYLINDER_RADIUS]);
                    break;
                case MSolidPrimitive.CONE:
                    Vector3 coneAxis = originRotation * Vector3.up * (float)message.dimensions[MSolidPrimitive.CONE_HEIGHT] * 0.5f;
                    drawing.DrawCone(originPosition - coneAxis, originPosition + coneAxis, color, (float)message.dimensions[MSolidPrimitive.CONE_RADIUS]);
                    break;
            }
        }

        public static void Draw<C>(this MTransform transform, BasicDrawing drawing, float size = 0.01f, bool drawUnityAxes = false) where C : ICoordinateSpace, new()
        {
            transform.rotation.Draw<C>(drawing, transform.translation.From<C>(), size, drawUnityAxes);
        }

        public static void Draw<C>(this MVector3 message, BasicDrawing drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 point = message.From<C>();
            drawing.DrawPoint(point, color, size);
            drawing.DrawLabel(label, point, color, size * 1.5f);
        }

        public static void Draw<C>(this MVector3 message, BasicDrawing drawing, Color color, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
        }

        public static void Draw<C>(this MVector3 message, BasicDrawing drawing, GameObject origin, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 point = message.From<C>();
            if (origin != null)
                point = origin.transform.TransformPoint(point);
            drawing.DrawPoint(point, color, size);
            drawing.DrawLabel(label, point, color, size * 1.5f);
        }



        public static void DrawAngularVelocityArrow(BasicDrawing drawing, Vector3 angularVelocity, Vector3 sphereCenter, Color32 color, float sphereRadius = 1.0f, float arrowThickness = 0.01f)
        {
            DrawRotationArrow(drawing, angularVelocity.normalized, angularVelocity.magnitude * Mathf.Rad2Deg, sphereCenter, color, sphereRadius, arrowThickness);
        }

        public static void DrawRotationArrow(BasicDrawing drawing, Quaternion rotation, Vector3 sphereCenter, Color32 color, float sphereRadius = 1.0f, float arrowThickness = 0.01f)
        {
            Vector3 axis;
            float angleDegrees;
            rotation.ToAngleAxis(out angleDegrees, out axis);
            DrawRotationArrow(drawing, axis, angleDegrees, sphereCenter, color, sphereRadius, arrowThickness);
        }

        public static void DrawRotationArrow(BasicDrawing drawing, Vector3 rotationAxis, float rotationDegrees, Vector3 sphereCenter, Color32 color, float sphereRadius = 1.0f, float arrowThickness = 0.01f)
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

            drawing.DrawLineStrip(color, arrowThickness, points.ToArray());
            drawing.DrawArrow(points[points.Count - 1], sphereCenter + currentRotation * Vector3.forward * sphereRadius, color, arrowThickness);
        }

        public static void GUI(this MAccel message)
        {
            message.linear.GUI("Linear");
            message.angular.GUI("Angular");
        }

        public static void GUI(this MBatteryState message)
        {
            GUILayout.Label($"Voltage: {message.voltage} (V)\nTemperature: {message.temperature} (ºC)\nCurrent: {message.current} (A)\nCharge: {message.charge} (Ah)\nCapacity: {message.capacity} (Ah)\nDesign Capacity: {message.design_capacity} (Ah)\nPercentage: {message.percentage}");
            GUILayout.Label($"Power supply status: {(MessageExtensions.BatteryStateStatusConstants)message.power_supply_status}");
            GUILayout.Label($"Power supply health: {(MessageExtensions.BatteryStateHealthConstants)message.power_supply_health}");
            GUILayout.Label($"Power supply technology: {(MessageExtensions.BatteryStateTechnologyConstants)message.power_supply_technology}");
            GUILayout.Label($"Present: {message.present}");
            GUILayout.Label($"Cell voltage: {String.Join(", ", message.cell_voltage)}");
            GUILayout.Label($"Cell temperature: {String.Join(", ", message.cell_temperature)}");
            GUILayout.Label($"Location: {message.location}\nSerial number: {message.serial_number}");
        }

        public static void GUI(this MColorRGBA message, bool withText = true)
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
        
        public static void GUI(this MCompressedImage message)
        {
            // TODO: Rescale/recenter image based on window height/width
            var img = message.ToTexture2D();
            var origRatio = (float)img.width / (float)img.height;
            UnityEngine.GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), img);
        }

        static string[] s_DiagnosticLevelTable = new string[]
        {
            "OK","WARN","ERROR","STALE"
        };

        public static void GUI(this MDiagnosticStatus message)
        {
            string status = (message.level >= 0 && message.level < s_DiagnosticLevelTable.Length) ? s_DiagnosticLevelTable[message.level] : "INVALID";
            GUILayout.Label($"Status of {message.name}|{message.hardware_id}: {status}");
            GUILayout.Label(message.message);
            foreach (MKeyValue keyValue in message.values)
            {
                GUILayout.Label($"   {keyValue.key}: {keyValue.value}");
            }
        }

        public static void GUI(this MDiagnosticArray message)
        {
            message.header.GUI();
            foreach (MDiagnosticStatus status in message.status)
                status.GUI();
        }

        public static void GUI(this MFluidPressure message)
        {
            GUILayout.Label($"Fluid Pressure: {message.fluid_pressure} (Pascals)\nVariance: {message.variance}");
        }

        static string[] s_GoalStatusTable = new string[]
        {
            "PENDING","ACTIVE","PREEMPTED","SUCCEEDED","ABORTED","REJECTED","PREEMPTING","RECALLING","RECALLED","LOST"
        };

        public static void GUI(this MGoalStatus message)
        {
            string status = (message.status >= 0 && message.status < s_GoalStatusTable.Length) ? s_GoalStatusTable[message.status] : $"INVALID({message.status})";
            GUILayout.Label($"Status: {message.goal_id} = {status}");
            GUILayout.Label(message.text);
        }

        public static void GUI(this MGoalStatusArray message)
        {
            message.header.GUI();
            foreach (MGoalStatus status in message.status_list)
                status.GUI();
        }

        public static void GUI(this MHeader message)
        {
            GUILayout.Label($"<{message.seq} {message.frame_id} {message.stamp.ToTimestampString()}>");
        }

        public static void GUI(this MIlluminance message)
        {
            GUILayout.Label($"Illuminance: {message.illuminance} (Lux)\nVariance: {message.variance}");
        }

        public static void GUI(this MImage message)
        {
            // TODO: Rescale/recenter image based on window height/width
            var img = message.ToTexture2D();
            var origRatio = (float)img.width / (float)img.height;
            UnityEngine.GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), img);
        }

        public static void GUI(this MInertia message)
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

        public static void GUI(this MJoy message, ref int layout)
        {
            string[] selStrings = {"DS4", "X360 Windows", "X360 Linux", "X360 (Wired)", "F710"};
            layout = GUILayout.SelectionGrid(layout, selStrings, 2);

            // Triggers
            GUILayout.BeginHorizontal();
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.LT, layout));
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.RT, layout));
            GUILayout.EndHorizontal();

            // Shoulders
            GUILayout.BeginHorizontal();
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.LB, layout));
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.RB, layout));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();

            // Dpad, central buttons
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.DPad, layout));
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.Back, layout));
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.Power, layout));
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.Start, layout));

            // N/E/S/W buttons
            GUILayout.BeginVertical();
            GUILayoutUtility.GetAspectRect(1);
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.BWest, layout));
            GUILayoutUtility.GetAspectRect(1);
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.BNorth, layout));
            GUILayoutUtility.GetAspectRect(1);
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.BSouth, layout));
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            GUILayoutUtility.GetAspectRect(1);
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.BEast, layout));
            GUILayoutUtility.GetAspectRect(1);
            GUILayout.EndVertical();

            GUILayout.EndHorizontal();

            // Joysticks
            GUILayout.BeginHorizontal();
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.LStick, layout));
            GUILayout.Box(message.TextureFromJoy(MessageExtensions.JoystickRegion.RStick, layout));
            GUILayout.EndHorizontal();
        }

        public static void GUI(this MJoyFeedback message)
        {
            GUILayout.Label($"Type: {(MessageExtensions.JoyFeedbackTypes)message.type}\nID: {message.id}\nIntensity: {message.intensity}");
        }

        public static void GUI(this MJoyFeedbackArray message)
        {
            foreach (MJoyFeedback m in message.array)
            {
                m.GUI();
            }
        }

        public static void GUI(this MMapMetaData message)
        {
            GUILayout.Label($"Load time: {message.map_load_time.ToTimestampString()}");
            GUILayout.Label($"Resolution: {message.resolution}");
            GUILayout.Label($"Size: {message.width}x{message.height}");
            message.origin.GUI();
        }

        public static void GUI(this MMesh message)
        {
            foreach (MPoint p in message.vertices)
                p.GUI();
            foreach (MMeshTriangle tri in message.triangles)
                tri.GUI();
        }

        public static void GUI(this MMeshTriangle message)
        {
            string text = "[" + String.Join(", ", message.vertex_indices.Select(i => i.ToString()).ToArray()) + "]";
            GUILayout.Label(text);
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

        public static void GUI(this MPolygon message)
        {
            GUILayout.Label($"({message.points.Length} points):");
            foreach (MPoint32 p in message.points)
                GUI(p);
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

        public static void GUI(this MRelativeHumidity message)
        {
            GUILayout.Label($"Relative Humidity: {message.relative_humidity}\nVariance: {message.variance}");
        }

        public static void GUI(this MSelfTestResponse message)
        {
            string pass = message.passed != 0 ? "OK" : "FAIL";
            GUILayout.Label($"Self test {message.id}: {pass}");
            foreach (MDiagnosticStatus status in message.status)
                status.GUI();
        }

        public static void GUI(this MSolidPrimitive message)
        {
            switch (message.type)
            {
                case MSolidPrimitive.BOX:
                    GUILayout.Label($"SolidPrimitive BOX\n[X:{message.dimensions[MSolidPrimitive.BOX_X]}, Y:{message.dimensions[MSolidPrimitive.BOX_Y]}, Z:{message.dimensions[MSolidPrimitive.BOX_Z]}]");
                    break;
                case MSolidPrimitive.SPHERE:
                    GUILayout.Label($"SolidPrimitive SPHERE\nRadius: {message.dimensions[MSolidPrimitive.SPHERE_RADIUS]}");
                    break;
                case MSolidPrimitive.CYLINDER:
                    GUILayout.Label($"SolidPrimitive CYLINDER\nHeight: {message.dimensions[MSolidPrimitive.CYLINDER_HEIGHT]}\nRadius: {message.dimensions[MSolidPrimitive.CYLINDER_RADIUS]}");
                    break;
                case MSolidPrimitive.CONE:
                    GUILayout.Label($"SolidPrimitive CONE\nHeight: {message.dimensions[MSolidPrimitive.CONE_HEIGHT]}\nRadius: {message.dimensions[MSolidPrimitive.CONE_RADIUS]}");
                    break;
                default:
                    GUILayout.Label($"INVALID shape {message.type}!?");
                    break;
            }
        }

        public static void GUI(this MTemperature message)
        {
            GUILayout.Label($"Temperature: {message.temperature} (ºC)\nVariance: {message.variance}");
        }

        public static void GUI(this MTime message)
        {
            GUILayout.Label(message.ToTimestampString());
        }

        public static void GUI(this MTimeReference message)
        {
            GUILayout.Label($"{message.time_ref.ToTimestampString()}\n{message.source}");
        }

        public static void GUI(this MTransform message)
        {
            message.translation.GUI("Translation");
            message.rotation.GUI("Rotation");
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

        static readonly GUIStyle s_ColorSwatchStyle = new GUIStyle
        {
            normal = new GUIStyleState { background = Texture2D.whiteTexture },
            fixedWidth = 25,
            fixedHeight = 25
        };

        public static void GUIMultiArray<T>(this MMultiArrayLayout layout, T[] data, ref bool tabulate)
        {
            tabulate = GUILayout.Toggle(tabulate, "Table view");
            GUIMultiArray(layout, data, tabulate);
        }

        static GUIStyle s_ArrayContainerStyle;

        public static void GUIMultiArray<T>(this MMultiArrayLayout layout, T[] data, bool tabulate = true)
        {
            if (s_ArrayContainerStyle == null)
            {
                s_ArrayContainerStyle = UnityEngine.GUI.skin.GetStyle("box");
            }
            GUIMultiArrayLevel(data, layout, layout.data_offset, 0, tabulate);
        }

        static void GUIMultiArrayLevel<T>(T[] data, MMultiArrayLayout layout, uint dataIndex, int depth, bool tabulate)
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