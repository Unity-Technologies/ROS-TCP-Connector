using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Reflection;
using System;
using System.Linq;

/*
public class MessageVisualizer<T>: ScriptableObject, IMessageVisualizer where T: RosMessageGeneration.Message
{
    public T message;

    public virtual void DrawGUI()
    {
        GUILayout.Label(message.ToString(), HUDPanel.messageStyle);
    }

    public virtual void Begin(RosMessageGeneration.Message message)
    {
        this.message = (T)message;
    }

    public virtual void End()
    {
    }
}
*/