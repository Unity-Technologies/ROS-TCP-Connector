using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class RegisterVisualizerAttribute : System.Attribute
    {
        public readonly string topic;
        public readonly int priority;

        public RegisterVisualizerAttribute(string topic = null)
        {
            this.topic = topic;
        }

        public RegisterVisualizerAttribute(int priority)
        {
            this.priority = priority;
        }
    }
}