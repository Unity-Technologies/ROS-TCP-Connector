using RosMessageTypes.Visualization;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class MarkerDefaultVisualizer : BaseVisualFactory<MarkerMsg>
    {
        public override bool CanShowDrawing => true;

        public override void Start()
        {
            base.Start();

            if (m_Topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<MarkerArrayMsg>(this, Priority);
            }
        }

        protected override IVisual CreateVisual(string topic)
        {
            return new MarkersVisual(topic);
        }

        public class MarkersVisual : IVisual
        {
            public string Label => $"Markers ({m_DrawingNamespaces.Count})";
            Dictionary<string, Dictionary<int, Drawing3d>> m_DrawingNamespaces = new Dictionary<string, Dictionary<int, Drawing3d>>();
            HashSet<string> m_HiddenNamespaces = new HashSet<string>();
            bool m_IsDrawingEnabled;
            public bool IsDrawingEnabled => m_IsDrawingEnabled;

            public MarkersVisual(string topic)
            {
                ROSConnection ros = ROSConnection.GetOrCreateInstance();
                RosTopicState state = ros.GetTopic(topic);
                if (state.RosMessageName == MarkerArrayMsg.k_RosMessageName)
                    ros.Subscribe<MarkerArrayMsg>(topic, OnMarkerArray);
                else if (state.RosMessageName == MarkerMsg.k_RosMessageName)
                    ros.Subscribe<MarkerMsg>(topic, OnMarker);
            }

            public void Destroy()
            {

            }

            public void OnGUI()
            {
                if (m_DrawingNamespaces.Count == 0)
                {
                    GUILayout.Label("Waiting for messages...");
                }

                foreach (KeyValuePair<string, Dictionary<int, Drawing3d>> element in m_DrawingNamespaces)
                {
                    string key = element.Key;
                    bool hidden = m_HiddenNamespaces.Contains(key);
                    bool newHidden = GUILayout.Toggle(hidden, $"{key} ({element.Value.Count})");
                    if (newHidden != hidden)
                    {
                        if (newHidden)
                        {
                            m_HiddenNamespaces.Remove(key);
                            foreach (Drawing3d drawing in element.Value.Values)
                            {
                                drawing.gameObject.SetActive(false);
                            }
                        }
                        else
                        {
                            m_HiddenNamespaces.Add(key);
                            foreach (Drawing3d drawing in element.Value.Values)
                            {
                                drawing.gameObject.SetActive(true);
                            }
                        }
                    }
                }
            }

            void OnMarkerArray(MarkerArrayMsg array)
            {
                foreach (MarkerMsg marker in array.markers)
                    OnMarker(marker);
            }

            void OnMarker(MarkerMsg marker)
            {
                Dictionary<int, Drawing3d> ns;
                Drawing3d drawing;
                switch (marker.action)
                {
                    case MarkerMsg.DELETEALL:
                        foreach (Dictionary<int, Drawing3d> namespaceToDestroy in m_DrawingNamespaces.Values)
                            foreach (Drawing3d drawingToDestroy in namespaceToDestroy.Values)
                                drawingToDestroy.Destroy();
                        m_DrawingNamespaces.Clear();
                        break;
                    case MarkerMsg.ADD:
                        if (!m_DrawingNamespaces.TryGetValue(marker.ns, out ns))
                        {
                            ns = new Dictionary<int, Drawing3d>();
                            m_DrawingNamespaces.Add(marker.ns, ns);
                        }
                        if (!ns.TryGetValue(marker.id, out drawing))
                        {
                            drawing = Drawing3d.Create();
                            ns.Add(marker.id, drawing);
                        }
                        if (marker.lifetime.sec == 0 && marker.lifetime.nanosec == 0)
                            drawing.ClearDuration();
                        else
                            drawing.SetDuration(marker.lifetime.sec + marker.lifetime.nanosec / 1E9f);
                        drawing.Clear();
                        Draw<FLU>(marker, drawing);
                        break;
                    case MarkerMsg.DELETE:
                        if (!m_DrawingNamespaces.TryGetValue(marker.ns, out ns))
                        {
                            ns = new Dictionary<int, Drawing3d>();
                            m_DrawingNamespaces.Add(marker.ns, ns);
                        }
                        if (ns.TryGetValue(marker.id, out drawing))
                        {
                            drawing.Destroy();
                            ns.Remove(marker.id);
                        }
                        break;
                }
            }

            public static void Draw<C>(MarkerMsg marker, Drawing3d drawing)
                where C : ICoordinateSpace, new()
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
                            drawing.DrawLineStrip(marker.points.Select(p => p.From<C>()).ToArray(), marker.colors.Select(c => (Color32)c.ToUnityColor()).ToArray(), (float)marker.scale.x);
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
                                for (int Idx = 0; Idx < marker.points.Length; ++Idx)
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
                                for (int Idx = 2; Idx < marker.points.Length; Idx += 3)
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
                                for (int Idx = 2; Idx < marker.points.Length; Idx += 3)
                                {
                                    drawing.DrawTriangle(
                                        marker.points[Idx - 2].From<C>(),
                                        marker.points[Idx - 1].From<C>(),
                                        marker.points[Idx].From<C>(),
                                        color);
                                }
                            }
                        }
                        break;
                }
            }

            public void SetDrawingEnabled(bool enabled)
            {
                if (m_IsDrawingEnabled == enabled)
                    return;

                foreach (var ns in m_DrawingNamespaces.Values)
                {
                    foreach (Drawing3d drawing in ns.Values)
                        drawing.enabled = enabled;
                }
                m_IsDrawingEnabled = enabled;
            }

            public void Redraw()
            {
                SetDrawingEnabled(true);
            }
        }
    }
}
