using System;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public abstract class VisualizerSettings : ScriptableObject
{
    public abstract string RosMessageName { get; }
}

public class VisualizerSettings<T> : VisualizerSettings
    where T : Message
{
    public override string RosMessageName => MessageRegistry.GetRosMessageName<T>();

    public virtual Action CreateGUI(T message, MessageMetadata meta)
    {
        return null;
    }

    public virtual void Draw(BasicDrawing drawing, T message, MessageMetadata meta)
    {
    }
}
