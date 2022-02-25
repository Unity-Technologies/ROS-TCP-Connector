using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class ImageGradientVisualizer : BaseVisualFactory<ImageMsg>
{
    [SerializeField]
    float m_GradientScale = 10;

    [SerializeField]
    Texture2D m_Gradient;

    [SerializeField]
    bool m_FlipY = true;

    public override bool CanShowDrawing => false;

    protected override IVisual CreateVisual(string topic)
    {
        return new ImageGradientVisual(topic, this);
    }

    public class ImageGradientVisual : IVisual
    {
        string m_Topic;

        ImageGradientVisualizer m_Factory;

        Texture2D m_Texture2D;
        Material m_GradientMaterial;

        public ImageGradientVisual(string topic, ImageGradientVisualizer factory)
        {
            m_Topic = topic;
            m_Factory = factory;
            m_GradientMaterial = new Material(Shader.Find("Unlit/ImageGradient"));
            m_GradientMaterial.SetTexture("_Gradient", m_Factory.m_Gradient);

            ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(m_Topic, AddMessage);
        }

        public virtual void AddMessage(Message message)
        {
            if (!VisualizationUtils.AssertMessageType<ImageMsg>(message, m_Topic))
                return;

            this.message = (ImageMsg)message;
            m_Texture2D = null;
        }

        public ImageMsg message { get; private set; }

        public void OnGUI()
        {
            if (message == null)
            {
                GUILayout.Label("Waiting for message...");
                return;
            }

            message.header.GUI();
            GUILayout.Label($"{message.width}x{message.height}, encoding: {message.encoding}");
            if (message.data.Length > 0)
            {
                m_GradientMaterial.SetFloat("_GradientScale", 1.0f / m_Factory.m_GradientScale);
                m_GradientMaterial.SetFloat("_flipY", m_Factory.m_FlipY ? 1.0f : 0.0f);
                GetTexture().GUITexture(m_GradientMaterial);
            }
        }

        public Texture2D GetTexture()
        {
            if (m_Texture2D == null)
            {
                m_Texture2D = (message != null && message.data.Length > 0) ? message.ToTexture2D() : null;
            }
            return m_Texture2D;
        }

        public bool IsDrawingEnabled => false;
        public void SetDrawingEnabled(bool enabled) { }
        public void Redraw() { }
    }
}
