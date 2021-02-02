using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VisualizeMessageAttribute : System.Attribute
{
    public readonly System.Type messageType;
    public readonly string topic;

    public VisualizeMessageAttribute(System.Type messageType)
    {
        this.messageType = messageType;
    }

    public VisualizeMessageAttribute(string topic)
    {
        this.topic = topic;
    }
}
