using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class VisualizerSettingsGeneric<T> : ScriptableObject
        where T : Message
    {
        public string RosMessageName => MessageRegistry.GetRosMessageName<T>();

        [SerializeField]
        protected TFTrackingSettings m_TFTrackingSettings = new TFTrackingSettings { type = TFTrackingType.Exact, tfTopic = "/tf" };
        public TFTrackingSettings TFTrackingSettings { get => m_TFTrackingSettings; set => m_TFTrackingSettings = value; }

        public virtual Action CreateGUI(T message, MessageMetadata meta)
        {
            return null;
        }

        public virtual void Draw(Drawing3d drawing, T message, MessageMetadata meta)
        {
        }
    }
}
