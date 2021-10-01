using System;
using System.Collections.Generic;
//using RosMessageTypes.Map;
using RosMessageTypes.Nav;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class OccupancyGridDefaultVisualizer : BaseVisualFactory<OccupancyGridMsg>
{
    static readonly int k_Color0 = Shader.PropertyToID("_Color0");
    static readonly int k_Color100 = Shader.PropertyToID("_Color100");
    static readonly int k_ColorUnknown = Shader.PropertyToID("_ColorUnknown");
    [SerializeField]
    Vector3 m_Offset = Vector3.zero;
    [SerializeField]
    Material m_Material;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;
    [Header("Cell Colors")]
    [SerializeField]
    Color m_Unoccupied = Color.white;
    [SerializeField]
    Color m_Occupied = Color.black;
    [SerializeField]
    Color m_Unknown = Color.clear;

    Dictionary<string, OccupancyGridVisual> m_BaseVisuals = new Dictionary<string, OccupancyGridVisual>();

    public override bool CanShowDrawing => true;

    public override IVisual GetOrCreateVisual(string topic)
    {
        OccupancyGridVisual baseVisual;
        if (m_BaseVisuals.TryGetValue(topic, out baseVisual))
            return baseVisual;

        baseVisual = new OccupancyGridVisual(topic, this);
        m_BaseVisuals.Add(topic, baseVisual);
        return baseVisual;
    }

    protected override IVisual CreateVisual(string topic) => throw new NotImplementedException();

    public class OccupancyGridVisual : IVisual
    {
        string m_Topic;
        Mesh m_Mesh;
        Material m_Material;
        Texture2D m_Texture;
        bool m_TextureIsDirty = true;
        bool m_IsDrawingEnabled;
        public bool IsDrawingEnabled => m_IsDrawingEnabled;
        float m_LastDrawingFrameTime = -1;

        Drawing3d m_Drawing;
        OccupancyGridDefaultVisualizer m_Settings;
        OccupancyGridMsg m_Message;

        public uint Width => m_Message.info.width;
        public uint Height => m_Message.info.height;

        public OccupancyGridVisual(string topic, OccupancyGridDefaultVisualizer settings)
        {
            m_Topic = topic;
            m_Settings = settings;

            ROSConnection.GetOrCreateInstance().Subscribe<OccupancyGridMsg>(m_Topic, AddMessage);
        }

        public void AddMessage(Message message)
        {
            if (!VisualizationUtils.AssertMessageType<OccupancyGridMsg>(message, m_Topic))
                return;

            m_Message = (OccupancyGridMsg)message;
            m_TextureIsDirty = true;

            if (m_IsDrawingEnabled && Time.time > m_LastDrawingFrameTime)
                Redraw();

            m_LastDrawingFrameTime = Time.time;
        }

        public void Redraw()
        {
            if (m_Mesh == null)
            {
                m_Mesh = new Mesh();
                m_Mesh.vertices = new[]
                { Vector3.zero, new Vector3(0, 0, 1), new Vector3(1, 0, 1), new Vector3(1, 0, 0) };
                m_Mesh.uv = new[] { Vector2.zero, Vector2.up, Vector2.one, Vector2.right };
                m_Mesh.triangles = new[] { 0, 1, 2, 2, 3, 0 };
            }

            if (m_Material == null)
            {
                m_Material = (m_Settings.m_Material != null) ? new Material(m_Settings.m_Material) : new Material(Shader.Find("Unlit/OccupancyGrid"));
            }
            m_Material.mainTexture = GetTexture();
            m_Material.SetColor(k_Color0, m_Settings.m_Unoccupied);
            m_Material.SetColor(k_Color100, m_Settings.m_Occupied);
            m_Material.SetColor(k_ColorUnknown, m_Settings.m_Unknown);

            var origin = m_Message.info.origin.position.From<FLU>();
            var rotation = m_Message.info.origin.orientation.From<FLU>();
            rotation.eulerAngles += new Vector3(0, -90, 0); // TODO: Account for differing texture origin
            var scale = m_Message.info.resolution;

            if (m_Drawing == null)
            {
                m_Drawing = Drawing3dManager.CreateDrawing();
            }
            else
            {
                m_Drawing.Clear();
            }

            m_Drawing.SetTFTrackingSettings(m_Settings.m_TFTrackingSettings, m_Message.header);
            // offset the mesh by half a grid square, because the message's position defines the CENTER of grid square 0,0
            Vector3 drawOrigin = origin - rotation * new Vector3(scale * 0.5f, 0, scale * 0.5f) + m_Settings.m_Offset;
            m_Drawing.DrawMesh(m_Mesh, drawOrigin, rotation,
                new Vector3(m_Message.info.width * scale, 1, m_Message.info.height * scale), m_Material);
        }

        public void DeleteDrawing()
        {
            if (m_Drawing != null)
            {
                m_Drawing.Destroy();
            }

            m_Drawing = null;
        }

        public Texture2D GetTexture()
        {
            if (!m_TextureIsDirty)
                return m_Texture;

            if (m_Texture == null)
            {
                m_Texture = new Texture2D((int)m_Message.info.width, (int)m_Message.info.height, TextureFormat.R8, true);
                m_Texture.wrapMode = TextureWrapMode.Clamp;
                m_Texture.filterMode = FilterMode.Point;
            }
            else if (m_Message.info.width != m_Texture.width || m_Message.info.height != m_Texture.height)
            {
                m_Texture.Resize((int)m_Message.info.width, (int)m_Message.info.height);
            }

            m_Texture.SetPixelData(m_Message.data, 0);
            m_Texture.Apply();
            m_TextureIsDirty = false;
            return m_Texture;
        }

        public void OnGUI()
        {
            if (m_Message == null)
            {
                GUILayout.Label("Waiting for message...");
                return;
            }

            m_Message.header.GUI();
            m_Message.info.GUI();
        }

        public void SetDrawingEnabled(bool enabled)
        {
            if (m_IsDrawingEnabled == enabled)
                return;

            m_IsDrawingEnabled = enabled;

            if (!enabled && m_Drawing != null)
            {
                m_Drawing.Clear();
            }

            if (enabled && m_Message != null)
            {
                Redraw();
            }
        }
    }
}
