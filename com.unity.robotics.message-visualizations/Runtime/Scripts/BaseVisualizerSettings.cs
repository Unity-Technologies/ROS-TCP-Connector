using System;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class BaseVisualizerSettings<T> : ScriptableObject
    where T : Message
{
    public string RosMessageName => MessageRegistry.GetRosMessageName<T>();

    [SerializeField]
    protected TFTrackingType m_TFTrackingType = TFTrackingType.Exact;
    public TFTrackingType TFTrackingType { get => m_TFTrackingType; set => m_TFTrackingType = value; }

    public virtual Action CreateGUI(T message, MessageMetadata meta)
    {
        return null;
    }

    public virtual void Draw(BasicDrawing drawing, T message, MessageMetadata meta)
    {
    }
}
