using RosMessageTypes.Std;
using System;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class DrawingVisualizerWithSettings<TMessage, TDrawingSettings> : DrawingVisualizer<TMessage>
        where TMessage : Message
        where TDrawingSettings : BaseVisualizerSettings<TMessage>
    {
        public const string ScriptableObjectsSettingsPath = "ScriptableObjects/";
        public abstract string DefaultScriptableObjectPath { get; }

        [SerializeField]
        TDrawingSettings m_Settings;
        public TDrawingSettings Settings { get => m_Settings; set => m_Settings = value; }

        void Awake()
        {
            if (m_Settings == null)
            {
                m_Settings = Resources.Load<TDrawingSettings>(DefaultScriptableObjectPath);
            }
        }

        void OnValidate()
        {
            if (m_Settings == null)
            {
                m_Settings = (TDrawingSettings)Resources.Load<ScriptableObject>(DefaultScriptableObjectPath);
            }
        }

        protected override IVisual CreateVisual()
        {
            return new DrawingVisual(this);
        }

        public override void Draw(BasicDrawing drawing, TMessage message, MessageMetadata meta)
        {
            m_Settings.Draw(drawing, message, meta);
        }

        public void Redraw()
        {
            // settings have changed - update the visualization
            foreach (IVisual visual in AllVisuals)
            {
                visual.CreateDrawing();
            }
        }
    }
}
