using System;
using System.Collections.Generic;
using RosMessageTypes.Map;
using RosMessageTypes.Nav;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class OccupancyGridDefaultVisualizer : IVisualFactory
{
    [SerializeField]
    protected string m_OccupancyGridTopic;
    public string OccupancyGridTopic { get => m_OccupancyGridTopic; set => m_OccupancyGridTopic = value; }

    public int Priority { get; set; }

    [SerializeField]
    string m_OccupancyGridUpdateTopic;
    [SerializeField]
    Vector3 m_Offset = Vector3.zero;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;

    public bool CanShowDrawing => true;

    Dictionary<string, OccupancyGridVisual> m_BaseVisuals = new Dictionary<string, OccupancyGridVisual>();
    Dictionary<string, OccupancyGridUpdateVisual> m_UpdateVisuals = new Dictionary<string, OccupancyGridUpdateVisual>();

    public IVisual GetOrCreateVisual(string topic)
    {
        OccupancyGridVisual baseVisual;
        if (m_BaseVisuals.TryGetValue(topic, out baseVisual))
            return baseVisual;
        OccupancyGridUpdateVisual updateVisual;
        if (m_UpdateVisuals.TryGetValue(topic, out updateVisual))
            return updateVisual;

        if (topic == m_OccupancyGridUpdateTopic && !string.IsNullOrEmpty(m_OccupancyGridTopic))
        {
            baseVisual = (OccupancyGridVisual)GetOrCreateVisual(m_OccupancyGridTopic);
            updateVisual = new OccupancyGridUpdateVisual(baseVisual);
            m_UpdateVisuals.Add(topic, updateVisual);
            return updateVisual;
        }

        const string updateSuffix = "_update";
        if (topic.EndsWith(updateSuffix))
        {
            string baseTopic = topic.Remove(topic.Length - updateSuffix.Length);
            if (!m_BaseVisuals.TryGetValue(baseTopic, out baseVisual))
            {
                baseVisual = new OccupancyGridVisual(this);
                m_BaseVisuals.Add(baseTopic, baseVisual);
            }
            updateVisual = new OccupancyGridUpdateVisual(baseVisual);
            m_UpdateVisuals.Add(topic, updateVisual);
            return updateVisual;
        }
        else
        {
            baseVisual = new OccupancyGridVisual(this);
            m_BaseVisuals.Add(topic, baseVisual);
            return baseVisual;
        }
    }

    public void Start()
    {
        if (string.IsNullOrEmpty(m_OccupancyGridTopic))
        {
            VisualFactoryRegistry.RegisterTypeVisualizer<OccupancyGridMsg>(this, Priority);
            VisualFactoryRegistry.RegisterTypeVisualizer<OccupancyGridUpdateMsg>(this, Priority);
        }
        else
        {
            VisualFactoryRegistry.RegisterTopicVisualizer(m_OccupancyGridTopic, this, Priority);
            if (!string.IsNullOrEmpty(m_OccupancyGridUpdateTopic))
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(m_OccupancyGridUpdateTopic, this, Priority);
            }
        }
    }

    public class OccupancyGridVisual : IVisual
    {
        Mesh m_Mesh;
        Material m_Material;
        Texture2D m_Texture;
        bool m_TextureIsDirty = true;
        BasicDrawing m_BasicDrawing;
        OccupancyGridDefaultVisualizer m_Settings;

        OccupancyGridMsg m_Message;
        MessageMetadata m_Meta;

        public uint Width => m_Message.info.width;
        public uint Height => m_Message.info.height;

        public OccupancyGridVisual(OccupancyGridDefaultVisualizer settings)
        {
            m_Settings = settings;
        }

        public void AddMessage(Message message, MessageMetadata meta)
        {
            if (!MessageVisualizationUtils.AssertMessageType<OccupancyGridMsg>(message, meta))
                return;

            m_Message = (OccupancyGridMsg)message;
            m_Meta = meta;
            m_TextureIsDirty = true;
        }

        public void AddUpdate(OccupancyGridUpdateMsg message)
        {
            int width = (int)message.width;
            int height = (int)message.height;

            Texture2D texture = GetTexture();
            Color32[] colorBlock = new Color32[width * height];
            for (int Idx = 0; Idx < message.data.Length; ++Idx)
                colorBlock[Idx] = new Color32((byte)message.data[Idx], 0, 0, 0);
            texture.SetPixels32(message.x, message.y, width, height, colorBlock);
            texture.Apply();
        }

        public void CreateDrawing()
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
                m_Material = new Material(Shader.Find("Unlit/OccupancyGrid"));
            }
            m_Material.mainTexture = GetTexture();

            var origin = m_Message.info.origin.position.From<FLU>();
            var rotation = m_Message.info.origin.orientation.From<FLU>();
            rotation.eulerAngles += new Vector3(0, -90, 0); // TODO: Account for differing texture origin
            var scale = m_Message.info.resolution;

            if (m_BasicDrawing == null)
            {
                m_BasicDrawing = BasicDrawingManager.CreateDrawing();
            }
            else
            {
                m_BasicDrawing.Clear();
            }

            m_BasicDrawing.SetTFTrackingSettings(m_Settings.m_TFTrackingSettings, m_Message.header);
            // offset the mesh by half a grid square, because the message's position defines the CENTER of grid square 0,0
            Vector3 drawOrigin = origin - rotation * new Vector3(scale * 0.5f, 0, scale * 0.5f) + m_Settings.m_Offset;
            m_BasicDrawing.DrawMesh(m_Mesh, drawOrigin, rotation,
                new Vector3(m_Message.info.width * scale, 1, m_Message.info.height * scale), m_Material);
        }

        public void DeleteDrawing()
        {
            if (m_BasicDrawing != null)
            {
                m_BasicDrawing.Destroy();
            }

            m_BasicDrawing = null;
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
            m_Message.header.GUI();
            m_Message.info.GUI();
        }
    }

    public class OccupancyGridUpdateVisual : IVisual
    {
        OccupancyGridVisual m_BaseVisual;
        public OccupancyGridUpdateVisual(OccupancyGridVisual baseVisual)
        {
            m_BaseVisual = baseVisual;
        }

        public void AddMessage(Message message, MessageMetadata meta)
        {
            m_BaseVisual.AddUpdate((OccupancyGridUpdateMsg)message);
        }

        public void CreateDrawing()
        {
        }

        public void DeleteDrawing()
        {
        }

        public void OnGUI()
        {
        }
    }
}
