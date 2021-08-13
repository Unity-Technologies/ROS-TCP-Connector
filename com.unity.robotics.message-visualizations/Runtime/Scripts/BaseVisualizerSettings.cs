using System;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class BaseVisualizerSettings<T> : ScriptableObject
    where T : Message
{
    public string RosMessageName => MessageRegistry.GetRosMessageName<T>();

    public virtual Action CreateGUI(T message, MessageMetadata meta)
    {
        return null;
    }

    public virtual void Draw(BasicDrawing drawing, T message, MessageMetadata meta)
    {
    }
}
