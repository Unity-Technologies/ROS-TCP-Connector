using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public interface IVisualizer
{
    void Register(int priority);
    object CreateDrawing(Message msg, MessageMetadata meta);
    void DeleteDrawing(object drawing);
    Action CreateGUI(Message msg, MessageMetadata meta, object drawing);
}

public class Visualizer<Msg> : MonoBehaviour, IVisualizer where Msg : Message
{
    public string topic;
    bool didRegister;

    public void Register(int priority)
    {
        if(topic == "")
            MessageVisualizations.RegisterVisualizer<Msg>(this, priority);
        else
            MessageVisualizations.RegisterVisualizer(topic, this, priority);
        didRegister = true;
    }

    public void Start()
    {
        if(!didRegister)
            Register(0);
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
        return MessageVisualizations.CreateDefaultGUI(msg, meta);
    }

    object IVisualizer.CreateDrawing(Message msg, MessageMetadata meta)
    {
        return CreateDrawing((Msg)msg, meta);
    }

    Action IVisualizer.CreateGUI(Message msg, MessageMetadata meta, object drawing)
    {
        return CreateGUI((Msg)msg, meta, drawing);
    }
}
