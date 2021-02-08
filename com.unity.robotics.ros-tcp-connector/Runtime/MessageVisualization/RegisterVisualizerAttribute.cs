using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RegisterVisualizerAttribute : System.Attribute
{
    public readonly string topic;

    public RegisterVisualizerAttribute(string topic = null)
    {
        this.topic = topic;
    }
}
