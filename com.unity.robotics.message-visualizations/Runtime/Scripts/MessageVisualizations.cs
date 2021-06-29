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

        public static void DrawAxisVectors<C>(BasicDrawing drawing, Vector3Msg position, QuaternionMsg rotation, float size, bool drawUnityAxes) where C : ICoordinateSpace, new()
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

        public static void Draw<C>(this AccelMsg message, BasicDrawing drawing, Color color, GameObject origin, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 originPos = (origin == null) ? Vector3.zero : origin.transform.position;
            drawing.DrawArrow(originPos, originPos + message.linear.From<C>() * lengthScale, color, thickness);
            DrawAngularVelocityArrow(drawing, message.angular.From<C>(), originPos, color, sphereRadius, thickness);
        }

        public static void Draw<C>(this GridCellsMsg message, BasicDrawing drawing, Color color, float radius = 0.01f) where C : ICoordinateSpace, new()
        {
            DrawPointCloud<C>(message.cells, drawing, color, radius);
        }

        public static void Draw<C>(this ImageMsg message, BasicDrawing drawing, Color color, PointMsg[] points, float radius = 0.01f) where C : ICoordinateSpace, new()
        {
            DrawPointCloud<C>(points, drawing, color, radius);
        }

        public static void Draw<C>(this ImuMsg message, BasicDrawing drawing, Color color, float lengthScale = 1, float sphereRadius = 1, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            TFFrame frame = TFSystem.instance.GetTransform(message.header);
            message.orientation.Draw<C>(drawing, frame.translation);
            drawing.DrawArrow(frame.translation, frame.translation + message.linear_acceleration.From<C>() * lengthScale, color, thickness);
            DrawAngularVelocityArrow(drawing, message.angular_velocity.From<C>(), frame.translation, color, sphereRadius, thickness);
        }

        public static void DrawPointCloud<C>(PointMsg[] points, BasicDrawing drawing, Color color, float radius = 0.01f) where C : ICoordinateSpace, new()
        {
            PointCloudDrawing pointCloud = drawing.AddPointCloud(points.Length);
            foreach (PointMsg p in points)
                pointCloud.AddPoint(p.From<C>(), color, radius);
            pointCloud.Bake();
        }

        public static void DrawPointCloud<C>(Point32Msg[] points, BasicDrawing drawing, Color color, float radius = 0.01f) where C : ICoordinateSpace, new()
        {
            PointCloudDrawing pointCloud = drawing.AddPointCloud(points.Length);
            foreach (Point32Msg p in points)
                pointCloud.AddPoint(p.From<C>(), color, radius);
            pointCloud.Bake();
        }

        public static void Draw<C>(this PointCloudMsg message, BasicDrawing drawing, Color color, PointCloudVisualizerSettings cConfs) where C : ICoordinateSpace, new()
        {
            message.Draw<C>(drawing.AddPointCloud(message.points.Length), color, cConfs);
        }

        public static void Draw<C>(this PointCloudMsg message, PointCloudDrawing pointCloud, Color color, PointCloudVisualizerSettings cConfs) where C : ICoordinateSpace, new()
        {
            pointCloud.SetCapacity(message.points.Length);
            TFFrame frame = TFSystem.instance.GetTransform(message.header);

            Dictionary<string, int> channelToIdx = new Dictionary<string, int>();
            for (int i = 0; i < message.channels.Length; i++)
            {
                channelToIdx.Add(message.channels[i].name, i);
            }

            for (int i = 0; i < message.points.Length; i++)
            {
                Vector3 worldPoint = frame.TransformPoint(message.points[i].From<FLU>());

                if (cConfs.m_UseRgbChannel)
                {
                    switch (cConfs.colorMode)
                    {
                        case ColorMode.HSV:
                            if (cConfs.m_RgbChannel.Length > 0)
                            {
                                float colC = message.channels[channelToIdx[cConfs.m_RgbChannel]].values[i];
                                color = Color.HSVToRGB(Mathf.InverseLerp(cConfs.m_RgbRange[0], cConfs.m_RgbRange[1], colC), 1, 1);
                            }
                            break;
                        case ColorMode.RGB:
                            if (cConfs.m_UseSeparateRgb)
                            {
                                if (cConfs.m_RChannel.Length > 0 && cConfs.m_GChannel.Length > 0 && cConfs.m_BChannel.Length > 0)
                                {
                                    var colR = Mathf.InverseLerp(cConfs.m_RRange[0], cConfs.m_RRange[1], message.channels[channelToIdx[cConfs.m_RChannel]].values[i]);
                                    var r = Mathf.InverseLerp(0, 1, colR);

                                    var colG = Mathf.InverseLerp(cConfs.m_GRange[0], cConfs.m_GRange[1], message.channels[channelToIdx[cConfs.m_GChannel]].values[i]);
                                    var g = Mathf.InverseLerp(0, 1, colG);

                                    var colB = Mathf.InverseLerp(cConfs.m_BRange[0], cConfs.m_BRange[1], message.channels[channelToIdx[cConfs.m_BChannel]].values[i]);
                                    var b = Mathf.InverseLerp(0, 1, colB);
                                    color = new Color(r, g, b, 1);
                                }
                            }
                            else
                            {
                                // uint8 (R,G,B) values packed into the least significant 24 bits, in order.
                                byte[] rgb = BitConverter.GetBytes(message.channels[channelToIdx[cConfs.m_RgbChannel]].values[i]);

                                var r = Mathf.InverseLerp(0, 1, BitConverter.ToSingle(rgb, 0));
                                var g = Mathf.InverseLerp(0, 1, BitConverter.ToSingle(rgb, 8));
                                var b = Mathf.InverseLerp(0, 1, BitConverter.ToSingle(rgb, 16));
                                color = new Color(r, g, b, 1);
                            }
                            break;
                    }
                }

                var radius = cConfs.m_Size;
                if (cConfs.m_UseSizeChannel && cConfs.m_SizeChannel.Length > 0)
                {
                    var size = message.channels[channelToIdx[cConfs.m_SizeChannel]].values[i];
                    radius = Mathf.InverseLerp(cConfs.m_SizeRange[0], cConfs.m_SizeRange[1], size);
                }

                pointCloud.AddPoint(worldPoint, color, radius);
            }
            pointCloud.Bake();
        }

        public static void Draw<C>(this PointCloud2Msg message, BasicDrawing drawing, Color color, PointCloud2VisualizerSettings cConfs) where C : ICoordinateSpace, new()
        {
            message.Draw<C>(drawing.AddPointCloud((int)(message.data.Length / message.point_step)), color, cConfs);
        }

        public static void Draw<C>(this PointCloud2Msg message, PointCloudDrawing pointCloud, Color color, PointCloud2VisualizerSettings cConfs) where C : ICoordinateSpace, new()
        {
            Dictionary<string, int> channelToIdx = new Dictionary<string, int>();
            for (int i = 0; i < message.fields.Length; i++)
            {
                channelToIdx.Add(message.fields[i].name, i);
            }

            pointCloud.SetCapacity((int)(message.data.Length / message.point_step));
            TFFrame frame = TFSystem.instance.GetTransform(message.header);

            int xChannelOffset = (int)message.fields[channelToIdx[cConfs.m_XChannel]].offset;
            int yChannelOffset = (int)message.fields[channelToIdx[cConfs.m_YChannel]].offset;
            int zChannelOffset = (int)message.fields[channelToIdx[cConfs.m_ZChannel]].offset;
            int rgbChannelOffset = (int)message.fields[channelToIdx[cConfs.m_RgbChannel]].offset;
            int rChannelOffset = (int)message.fields[channelToIdx[cConfs.m_RChannel]].offset;
            int gChannelOffset = (int)message.fields[channelToIdx[cConfs.m_GChannel]].offset;
            int bChannelOffset = (int)message.fields[channelToIdx[cConfs.m_BChannel]].offset;
            int sizeChannelOffset = (int)message.fields[channelToIdx[cConfs.m_SizeChannel]].offset;
            for (int i = 0; i < message.data.Length / message.point_step; i++)
            {
                int iPointStep = i * (int)message.point_step;
                var x = BitConverter.ToSingle(message.data, iPointStep + xChannelOffset);
                var y = BitConverter.ToSingle(message.data, iPointStep + yChannelOffset);
                var z = BitConverter.ToSingle(message.data, iPointStep + zChannelOffset);
                Vector3<C> localRosPoint = new Vector3<C>(x, y, z);
                Vector3 worldPoint = frame.TransformPoint(localRosPoint.toUnity);

                // TODO: Parse type based on PointField?
                if (cConfs.m_UseRgbChannel)
                {
                    switch (cConfs.colorMode)
                    {
                        case ColorMode.HSV:
                            if (cConfs.m_RgbChannel.Length > 0)
                            {
                                int colC = BitConverter.ToInt16(message.data, (iPointStep + rgbChannelOffset));
                                color = Color.HSVToRGB(Mathf.InverseLerp(cConfs.m_RgbRange[0], cConfs.m_RgbRange[1], colC), 1, 1);
                            }
                            break;
                        case ColorMode.RGB:
                            if (cConfs.m_RChannel.Length > 0 && cConfs.m_GChannel.Length > 0 && cConfs.m_BChannel.Length > 0)
                            {
                                var colR = Mathf.InverseLerp(cConfs.m_RRange[0], cConfs.m_RRange[1], BitConverter.ToSingle(message.data, iPointStep + rChannelOffset));
                                var r = Mathf.InverseLerp(0, 1, colR);

                                var colG = Mathf.InverseLerp(cConfs.m_GRange[0], cConfs.m_GRange[1], BitConverter.ToSingle(message.data, iPointStep + gChannelOffset));
                                var g = Mathf.InverseLerp(0, 1, colG);

                                var colB = Mathf.InverseLerp(cConfs.m_BRange[0], cConfs.m_BRange[1], BitConverter.ToSingle(message.data, iPointStep + bChannelOffset));
                                var b = Mathf.InverseLerp(0, 1, colB);
                                color = new Color(r, g, b, 1);
                            }
                            break;
                    }
                }

                var radius = cConfs.m_Size;

                if (cConfs.m_UseSizeChannel)
                {
                    var size = BitConverter.ToSingle(message.data, iPointStep + sizeChannelOffset);
                    radius = Mathf.InverseLerp(cConfs.m_SizeRange[0], cConfs.m_SizeRange[1], size);
                }

                pointCloud.AddPoint(worldPoint, color, radius);
            }
            //pointCloud.Bake();
        }

        public static void Draw<C>(this MagneticFieldMsg message, BasicDrawing drawing, Color color, float lengthScale = 1) where C : ICoordinateSpace, new()
        {
            drawing.DrawArrow(Vector3.zero, message.magnetic_field.From<C>() * lengthScale, color);
        }
        public static void Draw<C>(this LaserScanMsg message, BasicDrawing drawing, LaserScanVisualizerSettings cConfs) where C : ICoordinateSpace, new()
        {
            message.Draw<C>(drawing.AddPointCloud(message.ranges.Length), cConfs);
        }

        public static void Draw<C>(this LaserScanMsg message, PointCloudDrawing pointCloud, LaserScanVisualizerSettings cConfs) where C: ICoordinateSpace, new()
        {
            pointCloud.SetCapacity(message.ranges.Length);
            TFFrame frame = TFSystem.instance.GetTransform(message.header);
            // negate the angle because ROS coordinates are right-handed, unity coordinates are left-handed
            float angle = -message.angle_min;
            for (int i = 0; i < message.ranges.Length; i++)
            {
                var radius = cConfs.m_PointRadius;
                Vector3 localPoint = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0) * Vector3.forward * message.ranges[i];
                Vector3 worldPoint = frame.TransformPoint(localPoint);
                Color c = Color.HSVToRGB(Mathf.InverseLerp(message.range_min, message.range_max, message.ranges[i]), 1, 1);

                if (cConfs.m_UseIntensitySize && message.intensities.Length > 0)
                {
                    radius = Mathf.InverseLerp(cConfs.m_SizeRange[0], cConfs.m_SizeRange[1], message.intensities[i]);
                }
                pointCloud.AddPoint(worldPoint, c, radius);
                angle -= message.angle_increment;
            }
            pointCloud.Bake();
        }

        public static void Draw<C>(this MarkerMsg marker, BasicDrawing drawing) where C : ICoordinateSpace, new()
        {
            switch (marker.type)
            {
                case MarkerMsg.ARROW:
                    Vector3 startPoint;
                    Vector3 endPoint;
                    if (marker.points.Length >= 2)
                    {
                        startPoint = marker.points[0].From<C>();
                        endPoint = marker.points[1].From<C>();

                        float arrowheadGradient = 0.5f;
                        if (marker.scale.z != 0)
                            arrowheadGradient = (float)(marker.scale.y / marker.scale.z);

                        drawing.DrawArrow(startPoint, endPoint, marker.color.ToUnityColor(),
                            (float)marker.scale.x, (float)(marker.scale.y / marker.scale.x), arrowheadGradient);
                    }
                    else
                    {
                        startPoint = marker.pose.position.From<C>();
                        endPoint = startPoint + marker.pose.orientation.From<C>() * Vector3.forward * (float)marker.scale.x;

                        drawing.DrawArrow(startPoint, endPoint, marker.color.ToUnityColor(), (float)marker.scale.y);
                    }
                    break;
                case MarkerMsg.CUBE:
                    drawing.DrawCuboid(marker.pose.position.From<C>(), marker.scale.From<C>() * 0.5f, marker.pose.orientation.From<C>(), marker.color.ToUnityColor());
                    break;
                case MarkerMsg.SPHERE:
                    drawing.DrawSpheroid(marker.pose.position.From<C>(), marker.scale.From<C>() * 0.5f, marker.pose.orientation.From<C>(), marker.color.ToUnityColor());
                    break;
                case MarkerMsg.CYLINDER:
                    drawing.transform.position = marker.pose.position.From<C>();
                    drawing.transform.rotation = marker.pose.orientation.From<C>();
                    drawing.transform.localScale = marker.scale.From<C>();
                    drawing.DrawCylinder(new Vector3(0, -0.5f, 0), new Vector3(0, 0.5f, 0), marker.color.ToUnityColor(), 0.5f);
                    break;
                case MarkerMsg.LINE_STRIP:
                    drawing.transform.position = marker.pose.position.From<C>();
                    drawing.transform.rotation = marker.pose.orientation.From<C>();
                    if (marker.colors.Length == marker.points.Length)
                    {
                        drawing.DrawLineStrip(marker.points.Select(p => p.From<C>()).ToArray(), marker.colors.Select(c=>(Color32)c.ToUnityColor()).ToArray(), (float)marker.scale.x);
                    }
                    else
                    {
                        drawing.DrawLineStrip(marker.points.Select(p => p.From<C>()).ToArray(), marker.color.ToUnityColor(), (float)marker.scale.x);
                    }
                    break;
                case MarkerMsg.LINE_LIST:
                    drawing.transform.position = marker.pose.position.From<C>();
                    drawing.transform.rotation = marker.pose.orientation.From<C>();
                    if (marker.colors.Length == marker.points.Length)
                    {
                        drawing.DrawLines(marker.points.Select(p => p.From<C>()).ToArray(), marker.colors.Select(c => (Color32)c.ToUnityColor()).ToArray(), (float)marker.scale.x);
                    }
                    else
                    {
                        drawing.DrawLines(marker.points.Select(p => p.From<C>()).ToArray(), marker.color.ToUnityColor(), (float)marker.scale.x);
                    }
                    break;
                case MarkerMsg.CUBE_LIST:
                    {
                        drawing.transform.position = marker.pose.position.From<C>();
                        drawing.transform.rotation = marker.pose.orientation.From<C>();
                        Vector3 cubeScale = marker.scale.From<C>() * 0.5f;
                        if (marker.colors.Length == marker.points.Length)
                        {
                            for (int Idx = 0; Idx < marker.points.Length; ++Idx)
                            {
                                drawing.DrawCuboid(marker.points[Idx].From<C>(), cubeScale, marker.colors[Idx].ToUnityColor());
                            }
                        }
                        else
                        {
                            Color32 color = marker.color.ToUnityColor();
                            for (int Idx = 0; Idx < marker.points.Length; ++Idx)
                            {
                                drawing.DrawCuboid(marker.points[Idx].From<C>(), cubeScale, color);
                            }
                        }
                    }
                    break;
                case MarkerMsg.SPHERE_LIST:
                    {
                        drawing.transform.position = marker.pose.position.From<C>();
                        drawing.transform.rotation = marker.pose.orientation.From<C>();
                        Vector3 radii = marker.scale.From<C>() * 0.5f;
                        if (marker.colors.Length == marker.points.Length)
                        {
                            for (int Idx = 0; Idx < marker.points.Length; ++Idx)
                            {
                                drawing.DrawSpheroid(marker.points[Idx].From<C>(), radii, Quaternion.identity, marker.colors[Idx].ToUnityColor());
                            }
                        }
                        else
                        {
                            Color32 color = marker.color.ToUnityColor();
                            for (int Idx = 0; Idx < marker.points.Length; ++Idx)
                            {
                                drawing.DrawSpheroid(marker.points[Idx].From<C>(), radii, Quaternion.identity, color);
                            }
                        }
                    }
                    break;
                case MarkerMsg.POINTS:
                    {
                        PointCloudDrawing cloud = drawing.AddPointCloud(marker.points.Length);
                        cloud.transform.position = marker.pose.position.From<C>();
                        cloud.transform.rotation = marker.pose.orientation.From<C>();
                        float radius = (float)marker.scale.x;
                        if (marker.colors.Length == marker.points.Length)
                        {
                            for(int Idx = 0; Idx < marker.points.Length; ++Idx)
                            {
                                cloud.AddPoint(marker.points[Idx].From<C>(), marker.colors[Idx].ToUnityColor(), radius);
                            }
                        }
                        else
                        {
                            Color32 color = marker.color.ToUnityColor();
                            for (int Idx = 0; Idx < marker.points.Length; ++Idx)
                            {
                                cloud.AddPoint(marker.points[Idx].From<C>(), color, radius);
                            }
                        }
                        cloud.Bake();
                    }
                    break;
                case MarkerMsg.TEXT_VIEW_FACING:
                    drawing.DrawLabel(marker.text, marker.pose.position.From<C>(), marker.color.ToUnityColor());
                    break;
                case MarkerMsg.MESH_RESOURCE:
                    break;
                case MarkerMsg.TRIANGLE_LIST:
                    {
                        drawing.transform.position = marker.pose.position.From<C>();
                        drawing.transform.rotation = marker.pose.orientation.From<C>();
                        float radius = (float)marker.scale.x;
                        if (marker.colors.Length == marker.points.Length)
                        {
                            for (int Idx = 2; Idx < marker.points.Length; Idx+=3)
                            {
                                drawing.DrawTriangle(
                                    marker.points[Idx - 2].From<C>(),
                                    marker.points[Idx - 1].From<C>(),
                                    marker.points[Idx].From<C>(),
                                    marker.colors[Idx - 2].ToUnityColor(),
                                    marker.colors[Idx - 1].ToUnityColor(),
                                    marker.colors[Idx].ToUnityColor());
                            }
                        }
                        else
                        {
                            Color32 color = marker.color.ToUnityColor();
                            for (int Idx = 2; Idx < marker.points.Length; Idx+=3)
                            {
                                drawing.DrawTriangle(
                                    marker.points[Idx-2].From<C>(),
                                    marker.points[Idx-1].From<C>(),
                                    marker.points[Idx].From<C>(),
                                    color);
                            }
                        }
                    }
                    break;
            }
        }

        public static void Draw<C>(this MeshMsg message, BasicDrawing drawing, Color color, GameObject origin = null) where C : ICoordinateSpace, new()
        {
            Mesh mesh = new Mesh();
            mesh.vertices = message.vertices.Select(v => v.From<C>()).ToArray();
            mesh.triangles = message.triangles.SelectMany(tri => tri.vertex_indices.Select(i => (int)i)).ToArray();
            if (origin != null)
                drawing.DrawMesh(mesh, origin.transform, color);
            else
                drawing.DrawMesh(mesh, Vector3.zero, Quaternion.identity, Vector3.one, color);
        }

        public static void Draw<C>(this MultiEchoLaserScanMsg message, BasicDrawing drawing, MultiEchoLaserScanVisualizerSettings cConfs) where C : ICoordinateSpace, new()
        {
            message.Draw<C>(drawing.AddPointCloud(message.ranges.Length), cConfs);
        }

        public static void Draw<C>(this MultiEchoLaserScanMsg message, PointCloudDrawing pointCloud, MultiEchoLaserScanVisualizerSettings cConfs) where C: ICoordinateSpace, new()
        {
            pointCloud.SetCapacity(message.ranges.Length * message.ranges[0].echoes.Length);
            TFFrame frame = TFSystem.instance.GetTransform(message.header);
            // negate the angle because ROS coordinates are right-handed, unity coordinates are left-handed
            float angle = -message.angle_min;
            // foreach(MLaserEcho echo in message.ranges)
            for (int i = 0; i < message.ranges.Length; i++)
            {
                var echoes = message.ranges[i].echoes;
                // foreach (float range in echo.echoes)
                for (int j = 0; j < echoes.Length; j++)
                {
                    Vector3 localPoint = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0) * Vector3.forward * echoes[j];
                    Vector3 worldPoint = frame.TransformPoint(localPoint);
                    Color c = Color.HSVToRGB(Mathf.InverseLerp(message.range_min, message.range_max, echoes[j]), 1, 1);

                    var radius = cConfs.m_PointRadius;

                    if (message.intensities.Length > 0 && cConfs.m_UseIntensitySize)
                    {
                        radius = Mathf.InverseLerp(cConfs.m_SizeRange[0], cConfs.m_SizeRange[1], message.intensities[i].echoes[j]);
                    }

                    pointCloud.AddPoint(worldPoint, c, radius);
                }
                angle -= message.angle_increment;
            }
            pointCloud.Bake();
        }

        static Mesh s_OccupancyGridMesh;
        static Material s_OccupancyGridMaterial = new Material(Shader.Find("Unlit/Color"));

        public static void Draw<C>(this OccupancyGridMsg message, BasicDrawing drawing) where C : ICoordinateSpace, new()
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

        public static void Draw<C>(this PathMsg message, BasicDrawing drawing, Color color, float thickness = 0.1f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPath(message.poses.Select(pose => pose.pose.position.From<C>()), color, thickness);
        }

        public static void Draw<C>(this PlaneMsg message, BasicDrawing drawing, Color color, GameObject center = null, float size = 10.0f) where C : ICoordinateSpace, new()
        {
            message.Draw<C>(drawing, color, (center != null) ? center.transform.position : Vector3.zero, size);
        }

        public static void Draw<C>(this PlaneMsg message, BasicDrawing drawing, Color color, Vector3 origin, float size = 10.0f) where C : ICoordinateSpace, new()
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

        public static void Draw<C>(this PointMsg message, BasicDrawing drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void Draw<C>(this PointMsg message, BasicDrawing drawing, Color color, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
        }

        public static void Draw<C>(this Point32Msg message, BasicDrawing drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void Draw<C>(this Point32Msg message, BasicDrawing drawing, Color color, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
        }

        public static void Draw<C>(this PolygonMsg message, BasicDrawing drawing, Color color, float thickness = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 prevPos = message.points[message.points.Length - 1].From<C>();
            foreach (Point32Msg p in message.points)
            {
                Vector3 curPos = p.From<C>();
                drawing.DrawLine(prevPos, curPos, color, thickness);
                prevPos = curPos;
            }
        }

        public static void Draw<C>(this PoseMsg message, BasicDrawing drawing, float size = 0.1f, bool drawUnityAxes = false) where C : ICoordinateSpace, new()
        {
            DrawAxisVectors<C>(
                drawing,
                new Vector3Msg(message.position.x, message.position.y, message.position.z),
                message.orientation,
                size,
                drawUnityAxes
            );
        }

        public static void Draw<C>(this PoseArrayMsg message, BasicDrawing drawing, float size = 0.1f, bool drawUnityAxes = false) where C : ICoordinateSpace, new()
        {
            foreach (PoseMsg pose in message.poses)
            {
                pose.Draw<C>(drawing, size, drawUnityAxes);
            }
        }

        public static void Draw<C>(this QuaternionMsg message, BasicDrawing drawing, GameObject drawAtPosition = null, float size = 0.1f, bool drawUnityAxes = false)
    where C : ICoordinateSpace, new()
        {
            Vector3 position = drawAtPosition != null ? drawAtPosition.transform.position : Vector3.zero;
            DrawAxisVectors<C>(drawing, position.To<C>(), message, size, drawUnityAxes);
        }

        public static void Draw<C>(this QuaternionMsg message, BasicDrawing drawing, Vector3 position, float size = 0.1f, bool drawUnityAxes = false)
            where C : ICoordinateSpace, new()
        {
            DrawAxisVectors<C>(drawing, position.To<C>(), message, size, drawUnityAxes);
        }

        public static void Draw<C>(this RangeMsg message, BasicDrawing drawing, Color color, float size = 0.1f, bool drawUnityAxes = false)
            where C : ICoordinateSpace, new()
        {
            TFFrame frame = TFSystem.instance.GetTransform(message.header);

            var s = Mathf.Asin(message.field_of_view);
            var c = Mathf.Acos(message.field_of_view);
            Color col = Color.HSVToRGB(Mathf.InverseLerp(message.min_range, message.max_range, message.range), 1, 1);

            Vector3 end = new Vector3(message.range * c, 0, message.range * s);
            Matrix4x4 matrix = Matrix4x4.TRS(Vector3.zero, frame.rotation, Vector3.one);
            end = matrix.MultiplyPoint(end);

            drawing.DrawCone(frame.translation + end, frame.translation, col, Mathf.Rad2Deg * message.field_of_view / 2);
        }

        public static void Draw<C>(this SolidPrimitiveMsg message, BasicDrawing drawing, Color color, GameObject origin = null)
            where C : ICoordinateSpace, new()
        {
            Vector3 originPosition = origin != null ? origin.transform.position : Vector3.zero;
            Quaternion originRotation = origin != null ? origin.transform.rotation : Quaternion.identity;
            switch (message.type)
            {
                case SolidPrimitiveMsg.BOX:
                    drawing.DrawCuboid(
                        originPosition,
                        new Vector3<C>(
                            (float)message.dimensions[SolidPrimitiveMsg.BOX_X] * 0.5f,
                            (float)message.dimensions[SolidPrimitiveMsg.BOX_Y] * 0.5f,
                            (float)message.dimensions[SolidPrimitiveMsg.BOX_Z] * 0.5f).toUnity,
                        originRotation,
                        color
                    );
                    break;
                case SolidPrimitiveMsg.SPHERE:
                    drawing.DrawSphere(originPosition, color, (float)message.dimensions[SolidPrimitiveMsg.SPHERE_RADIUS]);
                    break;
                case SolidPrimitiveMsg.CYLINDER:
                    Vector3 cylinderAxis = originRotation * Vector3.up * (float)message.dimensions[SolidPrimitiveMsg.CYLINDER_HEIGHT] * 0.5f;
                    drawing.DrawCylinder(originPosition - cylinderAxis, originPosition + cylinderAxis, color, (float)message.dimensions[SolidPrimitiveMsg.CYLINDER_RADIUS]);
                    break;
                case SolidPrimitiveMsg.CONE:
                    Vector3 coneAxis = originRotation * Vector3.up * (float)message.dimensions[SolidPrimitiveMsg.CONE_HEIGHT] * 0.5f;
                    drawing.DrawCone(originPosition - coneAxis, originPosition + coneAxis, color, (float)message.dimensions[SolidPrimitiveMsg.CONE_RADIUS]);
                    break;
            }
        }

        public static void Draw<C>(this TransformMsg transform, BasicDrawing drawing, float size = 0.01f, bool drawUnityAxes = false) where C : ICoordinateSpace, new()
        {
            transform.rotation.Draw<C>(drawing, transform.translation.From<C>(), size, drawUnityAxes);
        }

        public static void Draw<C>(this Vector3Msg message, BasicDrawing drawing, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            Vector3 point = message.From<C>();
            drawing.DrawPoint(point, color, size);
            drawing.DrawLabel(label, point, color, size * 1.5f);
        }

        public static void Draw<C>(this Vector3Msg message, BasicDrawing drawing, Color color, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
        }

        public static void Draw<C>(this Vector3Msg message, BasicDrawing drawing, GameObject origin, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
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

        static string[] s_DiagnosticLevelTable = new string[]
        {
            "OK","WARN","ERROR","STALE"
        };

        public static void GUI(this DiagnosticStatusMsg message)
        {
            string status = (message.level >= 0 && message.level < s_DiagnosticLevelTable.Length) ? s_DiagnosticLevelTable[message.level] : "INVALID";
            GUILayout.Label($"Status of {message.name}|{message.hardware_id}: {status}");
            GUILayout.Label(message.message);
            foreach (MKeyValue keyValue in message.values)
            {
                GUILayout.Label($"   {keyValue.key}: {keyValue.value}");
            }
        }

        public static void GUI(this DiagnosticArrayMsg message)
        {
            message.header.GUI();
            foreach (DiagnosticStatusMsg status in message.status)
                status.GUI();
        }

        static string[] s_GoalStatusTable = new string[]
        {
            "PENDING","ACTIVE","PREEMPTED","SUCCEEDED","ABORTED","REJECTED","PREEMPTING","RECALLING","RECALLED","LOST"
        };

        public static void GUI(this GoalStatusMsg message)
        {
            string status = (message.status >= 0 && message.status < s_GoalStatusTable.Length) ? s_GoalStatusTable[message.status] : $"INVALID({message.status})";
            GUILayout.Label($"Status: {message.goal_id} = {status}");
            GUILayout.Label(message.text);
        }

        public static void GUI(this GoalStatusArrayMsg message)
        {
            message.header.GUI();
            foreach (GoalStatusMsg status in message.status_list)
                status.GUI();
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
            if (tex != null)
            {
                var origRatio = tex.width / (float)tex.height;
                UnityEngine.GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), tex);
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

        public static void GUI(this PoseMsg message)
        {
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

        public static void GUI(this RegionOfInterestMsg message, Texture2D tex)
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
                status.GUI();
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

        public static void GUI(this TimeMsg message)
        {
            GUILayout.Label(message.ToTimestampString());
        }

        public static void GUI(this TransformMsg message)
        {
            message.translation.GUI("Translation");
            message.rotation.GUI("Rotation");
        }

        public static void GUI(this TwistMsg message)
        {
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
