using RosMessageTypes.Std;
using System;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public abstract class DrawingVisualizerWithSettings<TMessage, TDrawingSettings> : DrawingVisualizer<TMessage>
        where TMessage : Message
        where TDrawingSettings : VisualizerSettingsGeneric<TMessage>
    {
        public const string ScriptableObjectsSettingsPath = "ScriptableObjects/";
        public abstract string DefaultScriptableObjectPath { get; }

        [SerializeField]
        TDrawingSettings m_Settings;
        public TDrawingSettings Settings { get => m_Settings; set => m_Settings = value; }

        public override string Name => (string.IsNullOrEmpty(m_Topic) ? "" : $"({m_Topic}) ") + Settings.name;

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

        public override void Draw(Drawing3d drawing, TMessage message, MessageMetadata meta)
        {
            m_Settings.Draw(drawing, message, meta);
        }

        public override Action CreateGUI(TMessage message, MessageMetadata meta)
        {
            return m_Settings.CreateGUI(message, meta);
        }

        public void Redraw()
        {
            // settings have changed - update the visualization
            foreach (IVisual visual in AllVisuals)
            {
                visual.Redraw();
            }
        }
    }
}
