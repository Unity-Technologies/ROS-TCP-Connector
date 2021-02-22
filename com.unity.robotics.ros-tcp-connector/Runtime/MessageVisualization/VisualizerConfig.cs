using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public interface IVisualizerConfig
{
    void Register(int priority);
    object CreateDrawing(Message msg, MessageMetadata meta);
    void DeleteDrawing(object drawing);
    Action CreateGUI(Message msg, MessageMetadata meta, object drawing);
}

public class VisualizerConfig<Msg> : MonoBehaviour, IVisualizerConfig where Msg : Message
{
    public string topic;

    public void Register(int priority)
    {
        if(topic == "")
            MessageVisualizations.RegisterVisualizer<Msg>(this, priority);
        else
            MessageVisualizations.RegisterVisualizer(topic, this, priority);
    }

    public virtual object CreateDrawing(Msg msg, MessageMetadata meta)
    {
        return null;
    }

    public virtual void DeleteDrawing(object drawing)
    {

    }

    public virtual Action CreateGUI(Msg msg, MessageMetadata meta, object drawing)
    {
        string text = msg.ToString();
        return () =>
        {
            GUILayout.Label(text);
        };
    }

    object IVisualizerConfig.CreateDrawing(Message msg, MessageMetadata meta)
    {
        return CreateDrawing((Msg)msg, meta);
    }

    Action IVisualizerConfig.CreateGUI(Message msg, MessageMetadata meta, object drawing)
    {
        return CreateGUI((Msg)msg, meta, drawing);
    }
}
